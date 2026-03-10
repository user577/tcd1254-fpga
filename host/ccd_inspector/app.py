"""TCD1254 CCD Inspector — application entry point."""

import logging
import sys

from PySide6.QtWidgets import QApplication

from ccd_inspector.gui.main_window import MainWindow


def main():
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s %(name)s %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    app = QApplication(sys.argv)
    app.setApplicationName("TCD1254 CCD Inspector")
    app.setStyle("Fusion")

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
