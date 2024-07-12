from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import QUrl
from PyQt5.QtWebEngineWidgets import QWebEngineView

class HTMLViewer(QMainWindow):
    def __init__(self, file_path, window_size: tuple[int, int] = (800,600)):
        super().__init__()
        self.setWindowTitle("HTML Viewer")
        self.setGeometry(100, 100, window_size[0], window_size[1])

        self.web_view = QWebEngineView()
        self.setCentralWidget(self.web_view)

        self.load_html_file(file_path)

    def load_html_file(self, file_path):
        try:
            self.web_view.setUrl(QUrl.fromLocalFile(file_path))
        except Exception as e:
            print("Error loading HTML file:", e)
