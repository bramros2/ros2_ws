import sys
import re
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QSlider, QVBoxLayout, QHBoxLayout, QPushButton, QMessageBox, \
    QFileDialog
from PyQt5.QtCore import Qt


class SettingsPage(QWidget):

    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):

        self.hmin = 0
        self.smin = 0
        self.vmin = 0
        self.hmax = 255
        self.smax = 255
        self.vmax = 255

        hmin_label = QLabel('Minimum Hue:')
        self.hmin_slider = QSlider()
        self.hmin_slider.setMinimum(0)
        self.hmin_slider.setMaximum(255)
        self.hmin_slider.setValue(self.hmin)
        self.hmin_slider.setOrientation(Qt.Horizontal)
        self.hmin_slider.valueChanged.connect(self.on_hmin_slider_changed)
        self.hmin_val_label = QLabel(str(self.hmin))

        smin_label = QLabel('Minimum Saturation:')
        self.smin_slider = QSlider()
        self.smin_slider.setMinimum(0)
        self.smin_slider.setMaximum(255)
        self.smin_slider.setValue(self.smin)
        self.smin_slider.setOrientation(Qt.Horizontal)
        self.smin_slider.valueChanged.connect(self.on_smin_slider_changed)
        self.smin_val_label = QLabel(str(self.smin))

        vmin_label = QLabel('Minimum Value:')
        self.vmin_slider = QSlider()
        self.vmin_slider.setMinimum(0)
        self.vmin_slider.setMaximum(255)
        self.vmin_slider.setValue(self.vmin)
        self.vmin_slider.setOrientation(Qt.Horizontal)
        self.vmin_slider.valueChanged.connect(self.on_vmin_slider_changed)
        self.vmin_val_label = QLabel(str(self.vmin))

        hmax_label = QLabel('Maximum Hue:')
        self.hmax_slider = QSlider()
        self.hmax_slider.setMinimum(0)
        self.hmax_slider.setMaximum(255)
        self.hmax_slider.setValue(self.hmax)
        self.hmax_slider.setOrientation(Qt.Horizontal)
        self.hmax_slider.valueChanged.connect(self.on_hmax_slider_changed)
        self.hmax_val_label = QLabel(str(self.hmax))

        smax_label = QLabel('Maximum Saturation:')
        self.smax_slider = QSlider()
        self.smax_slider.setMinimum(0)
        self.smax_slider.setMaximum(255)
        self.smax_slider.setValue(self.smax)
        self.smax_slider.setOrientation(Qt.Horizontal)
        self.smax_slider.valueChanged.connect(self.on_smax_slider_changed)
        self.smax_val_label = QLabel(str(self.smax))

        vmax_label = QLabel('Maximum Value:')
        self.vmax_slider = QSlider()
        self.vmax_slider.setMinimum(0)
        self.vmax_slider.setMaximum(255)
        self.vmax_slider.setValue(self.vmax)
        self.vmax_slider.setOrientation(Qt.Horizontal)
        self.vmax_slider.valueChanged.connect(self.on_vmax_slider_changed)
        self.vmax_val_label = QLabel(str(self.vmax))

        save_button = QPushButton('Save')
        save_button.clicked.connect(self.on_save_button_clicked)

        load_button = QPushButton('Load')
        load_button.clicked.connect(self.on_load_button_clicked)

        layout = QVBoxLayout()
        hmin_layout = QHBoxLayout()
        hmin_layout.addWidget(hmin_label, alignment=Qt.AlignRight)
        hmin_layout.addWidget(self.hmin_slider, alignment=Qt.AlignRight)
        hmin_layout.addWidget(self.hmin_val_label, alignment=Qt.AlignRight)
        layout.addLayout(hmin_layout)

        smin_layout = QHBoxLayout()
        smin_layout.addWidget(smin_label, alignment=Qt.AlignRight)
        smin_layout.addWidget(self.smin_slider, alignment=Qt.AlignRight)
        smin_layout.addWidget(self.smin_val_label, alignment=Qt.AlignRight)
        layout.addLayout(smin_layout)

        vmin_layout = QHBoxLayout()
        vmin_layout.addWidget(vmin_label, alignment=Qt.AlignRight)
        vmin_layout.addWidget(self.vmin_slider, alignment=Qt.AlignRight)
        vmin_layout.addWidget(self.vmin_val_label, alignment=Qt.AlignRight)
        layout.addLayout(vmin_layout)

        hmax_layout = QHBoxLayout()
        hmax_layout.addWidget(hmax_label, alignment=Qt.AlignRight)
        hmax_layout.addWidget(self.hmax_slider, alignment=Qt.AlignRight)
        hmax_layout.addWidget(self.hmax_val_label, alignment=Qt.AlignRight)
        layout.addLayout(hmax_layout)

        smax_layout = QHBoxLayout()
        smax_layout.addWidget(smax_label, alignment=Qt.AlignRight)
        smax_layout.addWidget(self.smax_slider, alignment=Qt.AlignRight)
        smax_layout.addWidget(self.smax_val_label, alignment=Qt.AlignRight)
        layout.addLayout(smax_layout)

        vmax_layout = QHBoxLayout()
        vmax_layout.addWidget(vmax_label, alignment=Qt.AlignRight)
        vmax_layout.addWidget(self.vmax_slider, alignment=Qt.AlignRight)
        vmax_layout.addWidget(self.vmax_val_label, alignment=Qt.AlignRight)
        layout.addLayout(vmax_layout)

        layout.addWidget(save_button)
        layout.addWidget(load_button)

        self.setLayout(layout)
        self.setGeometry(800,600,700,500)
        self.setFixedSize(800,600)
        self.setWindowTitle('Settings')

        self.hmin_slider.setFixedWidth(200)
        self.smin_slider.setFixedWidth(200)
        self.vmin_slider.setFixedWidth(200)
        self.hmax_slider.setFixedWidth(200)
        self.smax_slider.setFixedWidth(200)
        self.vmax_slider.setFixedWidth(200)

    def on_hmin_slider_changed(self, value):
        if self.hmax <= value:
            value = self.hmax -1
        self.hmin = value
        self.hmin_slider.setValue(self.hmin)
        self.hmin_val_label.setText(str(value))

    def on_smin_slider_changed(self, value):
        if self.smax <= value:
            value = self.smax -1
        self.smin = value
        self.smin_slider.setValue(self.smin)
        self.smin_val_label.setText(str(value))

    def on_vmin_slider_changed(self, value):
        if self.vmax <= value:
            value = self.vmax -1
        self.vmin = value
        self.vmin_slider.setValue(self.vmin)
        self.vmin_val_label.setText(str(value))

    def on_hmax_slider_changed(self, value):
        if self.hmin >= value:
            value = self.hmin +1
        self.hmax = value
        self.hmax_slider.setValue(self.hmax)
        self.hmax_val_label.setText(str(value))

    def on_smax_slider_changed(self, value):
        if self.smin >= value:
            value = self.smin +1
        self.smax = value
        self.smax_slider.setValue(self.smax)
        self.smax_val_label.setText(str(value))

    def on_vmax_slider_changed(self, value):
        if self.vmin >= value:
            value = self.vmin +1
        self.vmax = value
        self.vmax_slider.setValue(self.vmax)
        self.vmax_val_label.setText(str(value))

    def on_save_button_clicked(self):
        if self.hmin >= self.hmax or self.smin >= self.smax or self.vmin >= self.vmax:
            QMessageBox.warning(self, 'Error', 'Minimum value should be lower than maximum value')
        else:
            # get file name to save the settings
            file_name, _ = QFileDialog.getSaveFileName(self, 'Save Settings', '', 'Text files (*.txt)')

            # save settings to file
            if file_name:
                with open(file_name, 'w') as f:
                    f.write(f'hmin={self.hmin}\n')
                    f.write(f'smin={self.smin}\n')
                    f.write(f'vmin={self.vmin}\n')
                    f.write(f'hmax={self.hmax}\n')
                    f.write(f'smax={self.smax}\n')
                    f.write(f'vmax={self.vmax}')
                QMessageBox.information(self, 'Saved', 'Settings saved successfully.')

    def on_load_button_clicked(self):

        # Use file dialog to allow user to select a file
        file_path, _ = QFileDialog.getOpenFileName(self, 'Open Settings File', '', 'Text Files (*.txt)')

        # If a file was selected, read the values from the file and update the sliders
        if file_path:
            with open(file_path, 'r') as f:
                values = f.read().split('\n')
                if len(values) == 6:
                    try:
                        hmin = int(re.findall("\d+", values[0])[0])
                        smin = int(re.findall("\d+", values[1])[0])
                        vmin = int(re.findall("\d+", values[2])[0])
                        hmax = int(re.findall("\d+", values[3])[0])
                        smax = int(re.findall("\d+", values[4])[0])
                        vmax = int(re.findall("\d+", values[5])[0])

                        # Check that min values are lower than max values
                        if hmin < hmax and smin < smax and vmin < vmax:
                            self.hmin = hmin
                            self.smin = smin
                            self.vmin = vmin
                            self.hmax = hmax
                            self.smax = smax
                            self.vmax = vmax

                            # Update sliders
                            self.hmin_slider.setValue(hmin)
                            self.smin_slider.setValue(smin)
                            self.vmin_slider.setValue(vmin)
                            self.hmax_slider.setValue(hmax)
                            self.smax_slider.setValue(smax)
                            self.vmax_slider.setValue(vmax)

                            # Update labels
                            self.hmin_val_label.setText(str(hmin))
                            self.smin_val_label.setText(str(smin))
                            self.vmin_val_label.setText(str(vmin))
                            self.hmax_val_label.setText(str(hmax))
                            self.smax_val_label.setText(str(smax))
                            self.vmax_val_label.setText(str(vmax))
                        else:
                            QMessageBox.warning(self, "Invalid Values", "The values in the file are not valid.")
                    except ValueError:
                        QMessageBox.warning(self, "Invalid File", "The file does not contain valid integer values.")
                else:
                    QMessageBox.warning(self, "Invalid File", "The file does not contain the correct number of values.")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SettingsPage()
    window.show()
    sys.exit(app.exec_())