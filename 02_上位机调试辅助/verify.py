import os
import sys
import tempfile
from pathlib import Path

sys.dont_write_bytecode = True
os.environ['QT_QPA_PLATFORM'] = 'windows'
work = Path(tempfile.mkdtemp(prefix='mazemouse_verify_'))
os.environ['MAZEMOUSE_DATA_DIR'] = str(work / 'settings')
os.environ['MPLCONFIGDIR'] = str(work / 'mpl')

import csv
import json
import traceback
from PyQt5.QtWidgets import QApplication, QFileDialog, QMessageBox
import run

root = Path(__file__).resolve().parent
out = root.parent / '交付说明与验证' / '电脑端验证'
out.mkdir(parents=True, exist_ok=True)
application = QApplication([])
checks = []


def check(name, condition, details=None):
    checks.append({'name': name, 'pass': bool(condition), 'details': details})
    if not condition:
        raise AssertionError(name + ': ' + str(details))


try:
    window = run.create_window()
    window.resize(1400, 900)
    window.show()
    application.processEvents()
    check('主界面及六个页面创建', window.content_stack.count() == 6)
    for index in range(6):
        window.on_nav_item_clicked(index)
        application.processEvents()
        check('页面切换_' + str(index), window.content_stack.currentIndex() == index)
    window.on_nav_item_clicked(0)
    check('未实现的小车指令按钮禁用', not any(b.isEnabled() for b in [window.start_button, window.stop_button, window.reset_button, window.send_path_button]))
    source = run.attach_demo_source(window)
    frames = (root / '离线样例.txt').read_bytes().splitlines(keepends=True)
    source.feed(b'garbage\r\n' + frames[0][:8])
    check('不完整串口帧等待后续数据', window.frame_count == 0)
    source.feed(frames[0][8:] + frames[1])
    check('半包与两帧粘包恢复', window.frame_count == 2)
    malformed = [b's,8,0,0,0,1,1,1,1\n', b's,7,,0,0,1,1,1,1\n',
                 b's,7,0,5,0,1,1,1,1\n', b's,7,0,0,nan,1,1,1,1\n',
                 b's,7,0,0,0,2,1,1,1\n', b's,7,0,0,0,1,1,1,9\n']
    before = window.mouse_current_x, window.mouse_current_y
    for data in malformed:
        source.feed(data)
    check('六类非法帧被拒绝且位置不变', window.error_frame_count == 6 and before == (window.mouse_current_x, window.mouse_current_y))
    for frame in frames[2:]:
        source.feed(frame)
        application.processEvents()
    check('样例十一帧接收完整', window.frame_count == len(frames), window.frame_count)
    check('终点坐标显示', (window.mouse_current_x, window.mouse_current_y) == (4.5, 3.5))
    check('到达中心并保留原始路径', window.has_reached_goal and len(window.current_run_path) == len(frames))
    check('墙壁与传感器状态更新', bool(window.wall_map) and window.sensor_labels['前传感器:'].text() == '1')
    check('预估路径生成', bool(window.optimized_paths) and bool(window.get_best_optimized_path()))
    original = next(r for r in window.replay_runs if r['name'].startswith('原始轨迹'))
    check('原始回放包含终点', (original['path_x'][-1], original['path_y'][-1]) == (4.5, 3.5))
    count = len(window.optimized_paths)
    source.feed(frames[-1])
    check('重复终点帧不重复结算', len(window.optimized_paths) == count)
    window._update_data_status()
    application.processEvents()
    window.grab().save(str(out / '主界面_离线样例.png'))
    window.toggle_maze_view()
    application.processEvents()
    window.maze_plotter.draw()
    check('三维视图切换', window.maze_plotter.view_mode == '3D')
    window.toggle_maze_view()
    application.processEvents()
    check('二维视图恢复', window.maze_plotter.view_mode == '2D')
    replay = window.findChild(run.main.ReplayPage)
    replay.refresh_list()
    replay.list.setCurrentRow(next(i for i,r in enumerate(window.replay_runs) if r['name'].startswith('原始轨迹')))
    replay.draw_selected()
    QFileDialog.getSaveFileName = staticmethod(lambda *a, **k: (str(out / '离线样例轨迹.csv'), 'CSV'))
    QMessageBox.information = staticmethod(lambda *a, **k: QMessageBox.Ok)
    QMessageBox.warning = staticmethod(lambda *a, **k: QMessageBox.Ok)
    def export_error(*args, **kwargs):
        raise RuntimeError(str(args))
    QMessageBox.critical = staticmethod(export_error)
    replay.export_csv()
    rows = list(csv.DictReader((out / '离线样例轨迹.csv').open(encoding='utf-8')))
    check('CSV导出包含终点', rows[-1] == {'x': '4.5', 'y': '3.5'}, len(rows))
    QFileDialog.getSaveFileName = staticmethod(lambda *a, **k: (str(out / '离线样例轨迹.png'), 'PNG'))
    replay.export_png()
    check('PNG导出', (out / '离线样例轨迹.png').stat().st_size > 1000)
    window.on_nav_item_clicked(2)
    application.processEvents()
    window.grab().save(str(out / '回放界面_离线样例.png'))
    logpage = window.findChild(run.main.RealtimeLogPage)
    QFileDialog.getSaveFileName = staticmethod(lambda *a, **k: (str(out / '离线样例日志.txt'), 'TXT'))
    logpage.export_logs()
    check('串口日志导出', (out / '离线样例日志.txt').is_file() and 's,4,3' in (out / '离线样例日志.txt').read_text(encoding='utf-8'))
    window.close()
    status = 'PASS'
except BaseException:
    status = 'FAIL'
    checks.append({'name': 'exception', 'pass': False, 'details': traceback.format_exc()})
finally:
    result = {'status': status, 'scope': '离线字节流与桌面界面验证；不包含真实串口驱动、烧录或实车运动', 'checks': checks}
    (out / '验证结果.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False), flush=True)
sys.exit(0 if status == 'PASS' else 1)
