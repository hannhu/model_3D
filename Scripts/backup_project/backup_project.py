"""
Fusion 360 脚本: 项目备份 (Backup Active Project to Local Folder)

把当前活动项目里所有 F3D 文件导出到本地目录，保留云端的子文件夹结构。
- 同名文件会被最新的版本直接覆盖
- 当前活动文档直接 export，不会被 close/reopen
- 其他文档静默打开 export 后立即 close(saveChanges=False)，不会污染云端
"""

import os
import time
import traceback

import adsk.core
import adsk.fusion

DEFAULT_BACKUP_DIR = '/Users/hanhu/Documents/hh_forfun/model_3D/project_backup'


def _safe_filename(name: str) -> str:
    """删除路径分隔符等保留字符，保留中文。"""
    bad = '/\\:*?"<>|'
    out = ''.join('_' if c in bad else c for c in name)
    return out.strip() or 'unnamed'


def _collect_f3d_files(folder, rel=''):
    """递归遍历 DataFolder，返回 [(相对路径, DataFile), ...] 仅 F3D。"""
    out = []
    for i in range(folder.dataFiles.count):
        df = folder.dataFiles.item(i)
        ext = (df.fileExtension or '').lower()
        if ext == 'f3d':
            out.append((rel, df))
    for i in range(folder.dataFolders.count):
        sub = folder.dataFolders.item(i)
        sub_rel = os.path.join(rel, _safe_filename(sub.name)) if rel else _safe_filename(sub.name)
        out.extend(_collect_f3d_files(sub, sub_rel))
    return out


def _export_doc_as_f3d(doc, out_path):
    design = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    if not design:
        raise RuntimeError('document has no Design product')
    exp_mgr = design.exportManager
    opts = exp_mgr.createFusionArchiveExportOptions(out_path)
    exp_mgr.execute(opts)


def _resolve_project(app, ui):
    """获取项目，回避 app.data.activeProject 的 InternalValidationError。

    优先级:
      1. 当前活动文档的 parentProject (最稳)
      2. app.data.activeProject (新版本可能抛 InternalValidationError)
      3. 遍历 hubs 让用户选
    """
    # 1. 活动文档反查
    try:
        if app.activeDocument and app.activeDocument.dataFile:
            df = app.activeDocument.dataFile
            proj = getattr(df, 'parentProject', None)
            if proj is None and df.parentFolder:
                proj = df.parentFolder.parentProject
            if proj is not None:
                return proj
    except Exception:
        pass

    # 2. activeProject (可能抛错)
    try:
        proj = app.data.activeProject
        if proj is not None:
            return proj
    except Exception:
        pass

    # 3. 遍历 hubs，列出所有项目让用户选
    try:
        candidates = []
        hubs = app.data.dataHubs
        for i in range(hubs.count):
            hub = hubs.item(i)
            try:
                projs = hub.dataProjects
                for j in range(projs.count):
                    p = projs.item(j)
                    candidates.append((f'{hub.name} / {p.name}', p))
            except Exception:
                continue
        if not candidates:
            return None
        if len(candidates) == 1:
            return candidates[0][1]
        labels = '\n'.join(f'{i + 1}. {lbl}' for i, (lbl, _) in enumerate(candidates))
        result, ok = ui.inputBox(
            f'请输入要备份的项目编号:\n{labels}',
            '选择项目',
            '1',
        )
        if not ok:
            return None
        try:
            idx = int(result.strip()) - 1
            return candidates[idx][1]
        except Exception:
            return None
    except Exception:
        return None


def run(_context: str):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        # 1) 选目标目录
        dlg = ui.createFolderDialog()
        dlg.title = '选择备份目录（同名文件会被覆盖）'
        if os.path.isdir(DEFAULT_BACKUP_DIR):
            dlg.initialDirectory = DEFAULT_BACKUP_DIR
        if dlg.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        target_dir = dlg.folder
        os.makedirs(target_dir, exist_ok=True)

        # 2) 收集文件
        project = _resolve_project(app, ui)
        if not project:
            ui.messageBox(
                '无法定位活动项目。请确认已登录并打开了任一文档（脚本会反查其所属项目）。'
            )
            return
        try:
            root_folder = project.rootFolder
        except Exception as e:
            ui.messageBox(f'读取项目根目录失败: {e}')
            return
        files = _collect_f3d_files(root_folder)
        if not files:
            ui.messageBox(f'项目 [{project.name}] 中未找到 F3D 文件')
            return

        # 3) 二次确认
        ans = ui.messageBox(
            f'项目: {project.name}\n'
            f'F3D 文件数: {len(files)}\n'
            f'目标目录: {target_dir}\n\n'
            f'同名文件将被最新版本覆盖，是否继续？',
            '项目备份',
            adsk.core.MessageBoxButtonTypes.YesNoButtonType,
            adsk.core.MessageBoxIconTypes.QuestionIconType,
        )
        if ans != adsk.core.DialogResults.DialogYes:
            return

        # 4) 进度对话框
        progress = ui.createProgressDialog()
        progress.cancelButtonText = '取消'
        progress.isBackgroundTranslucent = False
        progress.isCancelButtonShown = True
        progress.show('项目备份', '初始化...', 0, len(files), 1)

        # 5) 记录原 active document，用于结束后恢复焦点
        active_doc = app.activeDocument
        active_id = None
        try:
            if active_doc and active_doc.dataFile:
                active_id = active_doc.dataFile.id
        except Exception:
            pass

        ok = 0
        errors = []
        t_start = time.time()

        for idx, (rel, df) in enumerate(files):
            if progress.wasCancelled:
                break
            progress.progressValue = idx
            progress.message = (
                f'{idx + 1}/{len(files)}  {df.name}.f3d'
            )
            adsk.doEvents()

            sub_dir = os.path.join(target_dir, rel) if rel else target_dir
            os.makedirs(sub_dir, exist_ok=True)
            out_path = os.path.join(sub_dir, _safe_filename(df.name) + '.f3d')

            # 强制覆盖（ExportManager 不允许直接覆盖现有文件）
            if os.path.exists(out_path):
                try:
                    os.remove(out_path)
                except Exception as e:
                    errors.append(f'{df.name}: 无法删除旧文件 ({e})')
                    continue

            doc = None
            opened_here = False
            try:
                if active_id and df.id == active_id:
                    doc = active_doc
                else:
                    doc = app.documents.open(df)
                    opened_here = True
                if not doc:
                    errors.append(f'{df.name}: 打开失败')
                    continue
                _export_doc_as_f3d(doc, out_path)
                ok += 1
                app.log(f'[backup] {df.name} -> {out_path}')
            except Exception as e:
                errors.append(f'{df.name}: {type(e).__name__}: {str(e)[:120]}')
            finally:
                if opened_here and doc is not None and doc is not active_doc:
                    try:
                        doc.close(False)
                    except Exception:
                        pass

        progress.hide()

        # 6) 尝试恢复原焦点（open 多个文档后焦点会跑掉）
        if active_doc:
            try:
                active_doc.activate()
            except Exception:
                pass

        elapsed = round(time.time() - t_start, 1)
        msg = (
            f'完成: {ok}/{len(files)} 个 F3D 文件\n'
            f'目录: {target_dir}\n'
            f'耗时: {elapsed}s'
        )
        if errors:
            msg += f'\n\n失败 {len(errors)} 个:'
            msg += '\n' + '\n'.join(errors[:15])
            if len(errors) > 15:
                msg += f'\n... 另有 {len(errors) - 15} 条已写入文本日志'
        ui.messageBox(msg)

    except Exception:
        ui.messageBox(f'出错:\n{traceback.format_exc()}')
