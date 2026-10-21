import os
import sys
from pathlib import Path

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent
DATA = Path(os.environ.get('MAZEMOUSE_DATA_DIR', str(ROOT / '运行数据')))
os.environ.setdefault('MPLCONFIGDIR', str(DATA / '绘图缓存'))

from PyQt5.QtCore import QByteArray, QCoreApplication, QObject, QSettings, QTimer, pyqtSignal
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QApplication, QAction, QMessageBox, QPushButton
import PyQt5
QCoreApplication.setLibraryPaths([str(Path(PyQt5.__file__).parent / 'Qt5' / 'plugins')])
import main


class DemoSource(QObject):
    readyRead = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.buffer = bytearray()
        self.opened = True

    def feed(self, data):
        self.buffer.extend(data)
        self.readyRead.emit()

    def readAll(self):
        data = QByteArray(bytes(self.buffer))
        self.buffer.clear()
        return data

    def isOpen(self):
        return self.opened

    def close(self):
        self.opened = False


def create_window(data_dir=None):
    folder = Path(data_dir or DATA)
    folder.mkdir(parents=True, exist_ok=True)

    def local_settings(*args, **kwargs):
        return QSettings(str(folder / 'settings.ini'), QSettings.IniFormat)

    main.QSettings = local_settings
    window = main.MicroMouseApp()
    window.resize(1400, 900)
    for button in window.findChildren(QPushButton):
        button.setStyleSheet('QPushButton { color: #172033; background-color: #e3efff; border: 1px solid #a7bfdc; border-radius: 5px; padding: 8px; } QPushButton:hover { background-color: #cbdfff; } QPushButton:disabled { color: #64748b; background-color: #e5e7eb; border-color: #cbd5e1; }')
        if '检查更新' in button.text():
            button.setEnabled(False)
            button.setToolTip('本交付为冻结版本，版本依据见交付说明。')
    for combo in (window.port_selector, window.baud_rate_selector, window.data_bits_selector,
                  window.stop_bits_selector, window.parity_selector):
        combo.setStyleSheet('QComboBox { color: #172033; background: white; padding: 4px; border: 1px solid #cbd5e1; border-radius: 5px; }')
    for label in window.sensor_labels.values():
        label.setMinimumHeight(40)
        label.setStyleSheet('QLabel { color: #1457a5; background: #eaf4ff; padding: 4px 10px; border: 1px solid #c5def8; border-radius: 5px; }')
    window.setWindowIcon(QIcon(str(ROOT / 'icon.ico')))
    window.auto_send_best_path = False
    menu = window.menuBar().addMenu('课设演示')
    demo = QAction('播放离线样例（不连接小车）', window)
    demo.triggered.connect(lambda: start_demo(window))
    menu.addAction(demo)
    window.statusBar().showMessage('数字量最终方案：串口接收监控；小车使用板载按键操作。')
    return window


def attach_demo_source(window):
    window.serial.close()
    window._port_refresh_timer.stop()
    source = DemoSource(window)
    window.serial = source
    source.readyRead.connect(window.read_serial_data)
    for widget in [window.connect_button, window.disconnect_button, window.port_selector,
                   window.send_button, window.send_data_input]:
        widget.setEnabled(False)
    window.act_connect.setEnabled(False)
    window.act_disconnect.setEnabled(False)
    window.setWindowTitle('电脑鼠上位机 | 离线样例演示 | 非实车数据')
    window._set_connection_state(False)
    window.statusBar().showMessage('离线演示：样例数据经过与串口相同的分帧和解析流程。重启软件可恢复串口监控。')
    return source


def start_demo(window):
    if getattr(window, '_demo_started', False):
        window.show_toast('本次样例已经开始，重新启动离线演示可再次播放。', 'info')
        return
    if window.serial.isOpen():
        window.show_toast('请先断开真实串口，再播放离线样例。', 'warning')
        return
    if window.frame_count:
        window.show_toast('请重新启动离线演示，避免与已有实车轨迹混合。', 'warning')
        return
    window._demo_started = True
    source = attach_demo_source(window)
    frames = (ROOT / '离线样例.txt').read_bytes().splitlines(keepends=True)
    timer = QTimer(window)
    window._demo_timer = timer
    iterator = iter(frames)

    def step():
        try:
            frame = next(iterator)
        except StopIteration:
            timer.stop()
            window.statusBar().showMessage('离线演示完成：到达(4,3)。可查看实时日志、轨迹回放，导出CSV和PNG。非实车验证。')
            return
        source.feed(frame)

    timer.timeout.connect(step)
    timer.start(500)


if __name__ == '__main__':
    application = QApplication(sys.argv)
    try:
        window = create_window()
        window.show()
        if '--demo' in sys.argv:
            QTimer.singleShot(300, lambda: start_demo(window))
        if '--smoke-exit' in sys.argv:
            QTimer.singleShot(8000, application.quit)
        sys.exit(application.exec_())
    except Exception:
        import traceback
        DATA.mkdir(parents=True, exist_ok=True)
        log = DATA / '启动错误.txt'
        log.write_text(traceback.format_exc(), encoding='utf-8')
        QMessageBox.critical(None, '启动失败', f'详细错误已保存到：\n{log}')
        raise
