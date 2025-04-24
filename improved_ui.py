import sys
import json
import time
import serial
import serial.tools.list_ports   # ← بعد import serial مباشرة
import csv
import configparser
from threading import Thread
import os
import random
import math
import re
import requests
from PyQt5 import QtWidgets, QtGui, QtCore, QtWebEngineWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.offsetbox import OffsetImage, AnnotationBbox   # ← أضف الاستيراد
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from matplotlib import patheffects
from datetime import datetime
from functools import partial
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QDialog, QFormLayout, QLineEdit, QDialogButtonBox, QVBoxLayout
from PyQt5.QtWebEngine import QtWebEngine
QtWebEngine.initialize()




##############################################
# دالة resource_path لتحديد مسار الموارد بشكل صحيح
##############################################

def resource_path(relative_path):
    """
    تعيد الدالة المسار الصحيح للملف سواء عند التشغيل من بيئة التطوير
    أو عند تشغيل الملف التنفيذي (exe) المُجمَّع بواسطة PyInstaller.
    """
    try:
        base_path = sys._MEIPASS  # مسار الملفات المؤقت عند تشغيل exe
    except Exception:
        base_path = os.path.abspath(".")
    return os.path.join(base_path, relative_path)

# ----------------------------------------------------------
# تحديد الكوكبة من رقم PRN طبقًا لجدول NMEA
# ----------------------------------------------------------
def classify_prn(prn: int) -> str:
    if   1   <= prn <=  32:   return "GP"
    elif 65  <= prn <=  96:   return "GL"
    elif 201 <= prn <= 237:   return "BD"
    elif 301 <= prn <= 336:   return "GA"
    elif 120 <= prn <= 158:   return "SB"
    elif prn in (193, 194):   return "QZ"
    elif prn in (398, 399):
        return "BD" if random.random() < 0.5 else "GI"



##############################################
# COMMUNICATION MODULE
##############################################
def openConnection(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Соединение открыто на {port} со скоростью {baudrate}")
        return ser
    except Exception as e:
        print("Ошибка открытия соединения:", e)
        return None

def closeConnection(ser):
    if ser:
        ser.close()
        print("Соединение закрыто")

def readData(ser):
    try:
        if ser and ser.is_open and ser.in_waiting:
            return ser.readline().decode('utf-8', errors='ignore')
    except serial.SerialException:
        # لو فيه مشكلة في الـ COM، نتجاهلها مؤقتاً
        return ""
    return ""

def sendCommand(ser, command_str):
    if ser:
        ser.write((command_str + "\r\n").encode("ascii"))  # CR+LF


##############################################
# PARSING MODULE
##############################################
def parseGGA(raw):
    parts = raw.strip().split(',')
    # على الأقل 10 حقول ضرورية
    if len(parts) < 10:
        return None

    # helper لتحويل آمن
    def to_float(s):
        try:
            return float(s)
        except (ValueError, TypeError):
            return None

    def to_int(s):
        try:
            return int(s)
        except (ValueError, TypeError):
            return None

    return {
        'time': parts[1],
        'latitude': parts[2],
        'lat_direction': parts[3],
        'longitude': parts[4],
        'lon_direction': parts[5],
        'fix_quality': parts[6],
        'num_satellites': to_int(parts[7]),
        'hdop': to_float(parts[8]),
        'altitude': to_float(parts[9]),
    }


def parseGSA(raw):
    parts = raw.strip().split(',')
    # نحتاج على الأقل 17 حقل، وثالث حقل هو fix type
    if len(parts) < 17 or parts[2] not in ('1', '2', '3'):
        return None

    # helper لتحويل آمن
    def to_float(s):
        try:
            # نفصل checksum إن وجدت
            s_clean = s.split('*')[0]
            return float(s_clean)
        except (ValueError, TypeError, IndexError):
            return None

    return {
        'mode1': parts[1],          # A/M
        'fix_type': parts[2],       # 1,2,3
        'pdop': to_float(parts[-3]),
        'hdop': to_float(parts[-2]),
        'vdop': to_float(parts[-1]),
    }


GSV_RE = re.compile(r'^\$(..)(GSV),(\d+),(\d+),(\d+),')

def parseGSV(raw_line):
    """
    تُعيد:
        talker    = 'GP' أو 'BD' أو 'GL' ...
        finished  = True عندما تكون هذه آخر رسالة فى السلسلة
        sats      = [dict(...), ...]
    """
    m = GSV_RE.match(raw_line)
    if not m:
        return None, False, []

    talker, _, total_msgs, msg_num, total_sats = m.groups()
    total_msgs, msg_num, total_sats = map(int, (total_msgs, msg_num, total_sats))

    parts = raw_line.strip().split(',')
    sats = []
    for i in range(4, len(parts) - 4, 4):
        # رقم القمر
        prn = parts[i].strip()

        # الارتفاع/السمت
        elev = parts[i + 1] or None
        azim = parts[i + 2] or None

        # قوّة الإشارة: ننظّف checksum وأى CR/LF، ثم نتحقق أنها رقم
        raw_snr = parts[i + 3].split('*')[0].strip() if parts[i + 3] else ""
        snr = raw_snr if raw_snr.isdigit() else None

        try:
            elev = float(elev) if elev is not None else None
            azim = float(azim) if azim is not None else None
        except ValueError:
            elev = azim = None
        try_int = int(prn) if prn.isdigit() else -1
        gnss = classify_prn(try_int) if talker == "GN" else talker

        sats.append(
            {'prn': prn, 'elevation': elev, 'azimuth': azim, 'snr': snr,
            'system': gnss}
        )

    finished = msg_num == total_msgs
    return talker, finished, sats

RMC_RE = re.compile(r'^\$(..)(RMC),')

def parseRMC(raw):
        """
        يُستخرج منها فقط الحقول الأساسية:
        parts[2] = status ('A' = active)
        parts[3],parts[4] = latitude, N/S
        parts[5],parts[6] = longitude, E/W
        """
        parts = raw.split(',')
        if len(parts) < 7 or parts[2] != 'A':
            return None
        return {
            'latitude':     parts[3],
            'lat_direction':parts[4],
            'longitude':    parts[5],
            'lon_direction':parts[6],
        }
##############################################
# LOGGING MODULE
##############################################
def writeLog(filePath, data):
    try:
        with open(filePath, 'a', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(data)
    except Exception as e:
        print("Ошибка записи лога:", e)

##############################################
# CONFIGURATION & LANGUAGE MODULE
##############################################
def loadConfiguration(configFile):
    config = configparser.ConfigParser()
    config.read(configFile, encoding="utf-8")
    return config

def loadLanguageResources(language):
    resources = {}
    try:
        with open(resource_path(os.path.join("MessageConfig", f"{language}.ini")), "r", encoding="utf-8") as f:
            for line in f:
                if '=' in line:
                    key, val = line.strip().split('=', 1)
                    resources[key] = val
    except Exception as e:
        print("Ошибка загрузки языковых ресурсов:", e)
    return resources

##############################################
# Matplotlib Integration مع PyQt - class MplCanvas
##############################################
""""
class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=3, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi, tight_layout=True)
        self.axes = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)
        
        # إعداد المحاور
        self.axes.set_xlim(0, 60)
        self.axes.set_ylim(0, 70)
        self.axes.set_yticks(range(0, 71, 10))
        self.axes.grid(True, linestyle='--', alpha=0.5)
        self.draw_sample_plot()

    def draw_sample_plot(self):
        x = list(range(61))
        y = [random.randint(10, 60) for _ in range(61)]
        self.axes.clear()
        self.axes.plot(x, y, linewidth=2, marker='o', label="Пример данных")
        self.axes.set_xlim(0, 60)
        self.axes.set_ylim(0, 70)
        self.axes.set_yticks(range(0, 71, 10))
        self.axes.grid(True, linestyle='--', alpha=0.5)
        self.axes.legend()
        self.draw()
"""

#-----------------------------------------------------------
# لوحة أعمدة SNR
#-----------------------------------------------------------
FREQ_COLORS = {
    'GP':'#ff3030', 'BD':'#c060ff', 'GL':'#00c8ff',
    'GA':'#9bff00', 'QZ':'#ffc000', 'SB':'#aaaaaa', 'GI':'#ff8c00'
}

class TrackingCanvas(FigureCanvas):
        
    BAR_WIDTH      = 0.65        # عـرض العمود (وحدة محور x ثابتة)
    MAX_SNR        = 60          # أقصى ارتفاع للمحور y
    COLOR_BY_SYS = {             # لون ثابت لكل كوكبة
        "GP": "#ff3030",         # GPS    (أحمر)
        "BD": "#ff3030",         # BeiDou نفس اللون إن أردته ثابتاً
        "GL": "#ff3030",
        "GA": "#ff3030",
        "QZ": "#ff3030",
        "SB": "#ff3030",
        "GI": "#ff3030",
        "GN": "#ff3030",
        "ALL": "#ff3030",        # للأوضاع المختلطة
    }

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(5,3), dpi=100, tight_layout=False)
        self.ax  = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)
        self.sats_by_sys = {}     # {'GP':[...], 'BD':[...]}
        self.current_sys = 'ALL'  # افتراضى

    # يُستدعى دورياً
    # -------------------------------------------------------
    #     ترسيم أعمدة الـ SNR مع حجم ثابت ووسم القيم
    # -------------------------------------------------------
    def update_plot(self):
        self.ax.clear()

        # -------- 1. اختيار الأقمار --------
        if self.current_sys == 'ALL':
            sats = [s for lst in self.sats_by_sys.values() for s in lst]
            slots = 63                       # نطاق شامل يكفى كل الأنظمة
        else:
            sats  = self.sats_by_sys.get(self.current_sys, [])
            # عدد الخانات الثابت لكل كوكبة (يمكن تعديله لاحقاً)
            slots = {
                "GP": 32, "BD": 63, "GL": 27,
                "GA": 36, "QZ":  5, "SB": 40, "GI": 14
            }.get(self.current_sys, 63)

        # استبعد أى SNR غير رقمى
        # فقط الأقمار ذات PRN و SNR رقميين
        sats = [
            s for s in sats
            if s.get('snr') and s['snr'].isdigit()
            and s.get('prn') and s['prn'].isdigit()
        ]
        sats.sort(key=lambda s: int(s['snr']), reverse=True)

        # -------- 2. تحويل إلى قوائم --------
        labels   = [s['prn'] for s in sats]
        values   = [int(s['snr']) for s in sats]
        xs       = range(len(values))
        bar_col  = "#ff3030"                # لون ثابت

        # -------- 3. رسم الأعمدة مع فراغ حقيقى --------
        SPACING = 3                             # كل عمود يبعد 1.5 وحدة
        xs = [i * SPACING for i in range(len(values))]

        BAR_W = 1.3                               # عرض العمود
        self.ax.bar(xs, values,
                    width=BAR_W,
                    color=bar_col,
                    edgecolor='black',
                    linewidth=0.5)

        # كتابة قيمة الـ SNR فوق العمود
        for x, val in zip(xs, values):
            self.ax.text(x, val + 1, str(val),
                         ha='center', va='bottom', fontsize=8)


        # -------- 4. إعداد المحاور --------
        self.ax.set_ylim(0, 65)              # أقصى ارتفاع
        self.ax.set_xlim(-0.5, slots - 0.5)  # مجال x ثابت
        self.ax.set_ylabel("SNR (dB‑Hz)")
        self.ax.set_xticks(xs)
        self.ax.set_xticklabels(labels, fontsize=8)
        self.ax.grid(axis='y', linestyle='--', alpha=.25)

        # -------- 5. المتوسّطات --------
        if values:
            avg  = sum(values) / len(values)
            top4 = sum(values[:4]) / min(4, len(values))
            self.ax.set_title(f"Average: {avg:.1f}   Top4: {top4:.1f}",
                              fontsize=9, loc='right')

        self.draw()




##############################################
# IconTextButton: زر بصورة ونص
##############################################
class IconTextButton(QtWidgets.QWidget):
    clicked = QtCore.pyqtSignal()

    def __init__(self, iconPath, text, parent=None):
        super().__init__(parent)
        self.iconPath = iconPath
        self.fullText = text
        self.expanded = False
        self.initUI()

    def initUI(self):
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(2)
        self.layout.setAlignment(QtCore.Qt.AlignCenter)
        self.iconLabel = QtWidgets.QLabel()
        # تحميل الصورة باستخدام resource_path
        pix = QtGui.QPixmap(resource_path(self.iconPath))
        self.iconLabel.setPixmap(pix.scaled(32, 32, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        self.iconLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.textLabel = QtWidgets.QLabel(self.fullText)
        self.textLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.textLabel.setWordWrap(True)
        self.textLabel.setVisible(False)
        self.layout.addWidget(self.iconLabel)
        self.layout.addWidget(self.textLabel)
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self.setFixedSize(60, 60)

    def setExpanded(self, expanded):
        self.expanded = expanded
        if expanded:
            self.textLabel.setVisible(True)
            self.setFixedSize(100, 100)
        else:
            self.textLabel.setVisible(False)
            self.setFixedSize(60, 60)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.clicked.emit()

##############################################
# PAGES
##############################################
#-----------------------------------------------------------
# زرّ علم قابل للنقر لاختيار كوكبة الأقمار
#-----------------------------------------------------------
#-----------------------------------------------------------
# زرّ علم + تسمية، يتبدّل لونه عند التحديد
#-----------------------------------------------------------
class FlagButton(QtWidgets.QWidget):
    clicked = QtCore.pyqtSignal(str)      # يرسل كود النظام عند الضغط

    def __init__(self, talker, icon_path, label, parent=None):
        super().__init__(parent)
        self.talker = talker
        self._selected = False

        # ========== الشكل ========== #
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(4, 2, 4, 2)
        lay.setSpacing(4)

        # الأيقونة
        pix = QtGui.QPixmap(resource_path(icon_path)).scaled(
                24, 24, QtCore.Qt.KeepAspectRatio,
                QtCore.Qt.SmoothTransformation)
        self.iconLbl = QtWidgets.QLabel()
        self.iconLbl.setPixmap(pix)
        lay.addWidget(self.iconLbl)

        # النصّ
        self.textLbl = QtWidgets.QLabel(label)
        self.textLbl.setStyleSheet("font-size: 10pt;")
        lay.addWidget(self.textLbl)

        self.setCursor(QtCore.Qt.PointingHandCursor)
        self._update_style()

    # ========== الواجهة ========== #
    def setSelected(self, val: bool):
        self._selected = val
        self._update_style()

    # ========== الداخل ========== #
    def _update_style(self):
        if self._selected:
            self.setStyleSheet("background:#dfffe0; border:2px solid #00aa00;"
                               "border-radius:4px;")
        else:
            self.setStyleSheet("background:transparent; border:none;")

    def mouseReleaseEvent(self, e):
        if e.button() == QtCore.Qt.LeftButton:
            self.clicked.emit(self.talker)


class Поток_данных(QtWidgets.QWidget):
    """صفحة بثّ البيانات – تُشبه Terminal وتعرض أحدث إحداثيات GNSS."""

    # إشارة داخلية لتمرير كل سطر يُستقبَل من الـ COM إلى واجهة المستخدم
    newLine = QtCore.pyqtSignal(str)
    locationChanged = QtCore.pyqtSignal(float, float, float)  # lat, lon, alt
    locationDetailed = QtCore.pyqtSignal(dict)
    

    # ---------------------------------------------
    # قائمة الأوامر السريعة
    # ---------------------------------------------
    QUICK_CMDS = [
    
        # يمكنك إضافة أي أمر جديد هنا
    ]

    def showConfigCom2Dialog(self):
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("Конфигурация COM2")
        dlg.setModal(True)

        vlay = QtWidgets.QVBoxLayout(dlg)
        vlay.addWidget(QtWidgets.QLabel("Укажите скорость:", dlg))

        combo = QtWidgets.QComboBox(dlg)
        combo.addItems(["9600","19200","38400","57600","115200"])
        vlay.addWidget(combo)

        btns = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel,
            parent=dlg
        )
        vlay.addWidget(btns)
        btns.accepted.connect(dlg.accept)
        btns.rejected.connect(dlg.reject)

        if dlg.exec_() == QtWidgets.QDialog.Accepted:
            speed = combo.currentText()
            cmd = f"Config com2 {speed}"
            # بدل وضع النص في self.input نرسل وننتظر ACK
            self.sendAndWaitUntilAck(cmd, timeout=1.0)

    def sendGGA1Commands(self):
        raws = ["GPGGA 1", "GPGSA 1", "GPGST 1", "GPGSV 1", "GPRMC 1"]
        for cmd in raws:
            self.sendAndWaitUntilAck(cmd, timeout=1.0)
            # يمكن إضافة msleep بسيط هنا إذا أحببت:
            QThread.msleep(10)


    def __init__(self, parent=None):
        super().__init__(parent)
        self._is_searching = False
        self._running = True     # ← سنستخدمه للتحكّم فى حلقة القراءة
        self.destroyed.connect(lambda _: setattr(self, "_running", False))
        self._last_info = {
        'lat': None, 'lon': None, 'alt': None,
        'fix_quality': None, 'num_sat': None,
        'pdop': None, 'hdop': None, 'vdop': None
    }
        # === تخطيط الصفحة ====================================================
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # 1) شريط الإحداثيات الحيّ
        self.coordLabel = QtWidgets.QLabel("Lat: --  |  Lon: --  |  Alt: --")
        self.coordLabel.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        self.coordLabel.setStyleSheet(
            "background:#1e1e1e; color:#00ff7f; padding:4px; font-family:Consolas;"
        )
        layout.addWidget(self.coordLabel)

        # 2) نافذة الترمينال (إخراج فقط)
        self.terminal = QtWidgets.QPlainTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setStyleSheet(
            "background:#1e1e1e; color:#dcdcdc; font-family:Consolas;"
        )
        layout.addWidget(self.terminal, 1)

        # 3) حاوية الإدخال (فوق) + صف الأزرار (تحت)
        cmdContainer = QtWidgets.QVBoxLayout()
        cmdContainer.setContentsMargins(0, 0, 0, 0)
        cmdContainer.setSpacing(4)

        # —– 3.1 حقل الكتابة
        self.input = QtWidgets.QLineEdit()
        self.input.setPlaceholderText("Введите команду здесь и нажмите Enter для отправки…")
        self.input.setStyleSheet(
            "background:#252526; color:#d4d4d4; font-family:Consolas;"
        )
        cmdContainer.addWidget(self.input)

        # —– 3.2 صف الأزرار السريعة
        btnRow = QtWidgets.QHBoxLayout()
        btnRow.setContentsMargins(0, 0, 0, 0)
        btnRow.setSpacing(4)
        btnModeBase = QtWidgets.QPushButton("Ввод координат")
        btnModeBase.setFixedHeight(26)
        btnModeBase.clicked.connect(self.showModeBaseDialog)
        btnRow.addWidget(btnModeBase)

        # زر Self‑optimize
        btnSelfOpt = QtWidgets.QPushButton("Авто фикс")
        btnSelfOpt.setFixedHeight(26)
        btnSelfOpt.clicked.connect(self.showSelfOptimizeDialog)
        btnRow.addWidget(btnSelfOpt)

        btnRtcm = QtWidgets.QPushButton("Вывод данных")
        btnRtcm.setFixedHeight(26)
        btnRtcm.clicked.connect(self.sendRtcmCommands)
        btnRow.addWidget(btnRtcm)

        # أزرار QUICK_CMDS
        for label, cmd in self.QUICK_CMDS:
            btn = QtWidgets.QPushButton(label)
            btn.setFixedHeight(26)
            btn.clicked.connect(lambda _, c=cmd: self.input.setText(c))
            btnRow.addWidget(btn)

        # زر Config com2
        btnConfig = QtWidgets.QPushButton("Скорость com2")
        btnConfig.setFixedHeight(26)
        btnConfig.clicked.connect(self.showConfigCom2Dialog)
        btnRow.addWidget(btnConfig)

        cmdContainer.addLayout(btnRow)
        layout.addLayout(cmdContainer)

        # زر GPGGA 1 (يرسل 5 أوامر دفعة)
        btnGGA1 = QtWidgets.QPushButton("NMEA")
        btnGGA1.setFixedHeight(26)
        btnGGA1.clicked.connect(self.sendGGA1Commands)
        btnRow.addWidget(btnGGA1)


        btnSave = QtWidgets.QPushButton("Сохранить")
        btnSave.setFixedHeight(26)
        btnSave.clicked.connect(self.sendSaveConfig)
        btnRow.addWidget(btnSave)



        # ربط الإشارات
        self.newLine.connect(self._append_line)
        self.input.returnPressed.connect(self._on_input_entered)

        # مقبض المنفذ التسلسلي (سيملؤه MainWindow لاحقاً)
        self.ser = None
        self._thread = None          # سيُحفظ فيه الخيط

    def sendSaveConfig(self):
        """
        يرسل الأمر 'saveconfig' ويكرر المحاولة حتى يستلم ACK.
        """
        self.sendAndWaitUntilAck("saveconfig", timeout=1.0) 

    def sendRtcmCommands(self):
        """
        يرسل دفعة أوامر RTCM وينتظر ACK لكل أمر قبل الانتقال.
        """
        cmds = [
            "config pvtalg multi",
            "RTCM1006 COM2 10",
            "RTCM1033 COM2 10",
            "RTCM1074 COM2 1",
            "RTCM1084 COM2 1",
            "RTCM1094 COM2 1",
            "RTCM1114 COM2 1",
            "RTCM1124 COM2 1",
        ]
        for cmd in cmds:
            # sendAndWaitUntilAck سيعرض في التيرمينال (> cmd)
            # ويكرر إرسال cmd حتى يستلم ACK ($command,…)
            self.sendAndWaitUntilAck(cmd, timeout=1.0)

    def showModeBaseDialog(self):
        dlg = QDialog(self)
        dlg.setWindowTitle("Ввод координат")
        vlay = QVBoxLayout(dlg)

        form = QFormLayout()
        latEdit  = QLineEdit(); latEdit.setPlaceholderText("0")
        lonEdit  = QLineEdit(); lonEdit.setPlaceholderText("0")
        altEdit  = QLineEdit(); altEdit.setPlaceholderText("0")
        baseEdit = QLineEdit(); baseEdit.setPlaceholderText("0‑4095 (необязательно)")

        form.addRow("Latitude (degrees):",  latEdit)
        form.addRow("Longitude (degrees):", lonEdit)
        form.addRow("Altitude (metre):",    altEdit)
        form.addRow("Base station ID:",     baseEdit)

        vlay.addLayout(form)

        btns = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=dlg
        )
        btns.accepted.connect(dlg.accept)
        btns.rejected.connect(dlg.reject)
        vlay.addWidget(btns)

        if dlg.exec_() == QDialog.Accepted:
            lat  = latEdit.text().strip()
            lon  = lonEdit.text().strip()
            alt  = altEdit.text().strip()
            base = baseEdit.text().strip()

            if not (lat and lon and alt):
                # يمكنك هنا إظهار رسالة خطأ للمستخدم
                return

            # بناء الأمر
            cmd = f"MODE BASE {lat} {lon} {alt}"
            if base:
                cmd += f" {base}"

            # إرسال وانتظار ACK
            self.sendAndWaitUntilAck(cmd, timeout=1.0)

    def sendAndWaitUntilAck(self, cmd: str, timeout: float = 1.0):

        if cmd.startswith("MODE BASE"):
            self._is_searching = True

        if not self.ser or not self.ser.is_open:
            return
        while True:
            # ← نظّف بافر الاستقبال بأمان
            try:
                self.ser.reset_input_buffer()
            except (serial.SerialException, serial.SerialTimeoutException):
                return

            sendCommand(self.ser, cmd)
            try:    self.newLine.emit("> " + cmd)
            except: pass

            start = time.time()
            while time.time() - start < timeout:
                if not self.ser or not self.ser.is_open:
                    return
                line = readData(self.ser).strip()
                if line.startswith("$command") and cmd in line:
                    try: self.newLine.emit(line)
                    except: pass
                    return
                QtCore.QThread.msleep(1)
            # لو انتهت المهلة بدون ACK تعود الحلقة لإعادة الإرسال


    def showSelfOptimizeDialog(self):
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("Автопозиционирование")
        vlay = QtWidgets.QVBoxLayout(dlg)

        form = QtWidgets.QFormLayout()
        # 1) مدة المتوسط بالثواني
        durEdit   = QtWidgets.QLineEdit(); durEdit.setText("120")
        # 2) حد المسافة (0–10 m)
        rangeEdit = QtWidgets.QLineEdit(); rangeEdit.setText("1")
        # 3) Base station ID (0–4095) необязательно
        baseEdit  = QtWidgets.QLineEdit(); baseEdit.setPlaceholderText("0–4095 (необязательно)")

        form.addRow("Max duration (s):",        durEdit)
        form.addRow("Distance limit (m):",      rangeEdit)
        form.addRow("Base station identifier:", baseEdit)
        vlay.addLayout(form)

        btns = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel,
            parent=dlg
        )
        btns.accepted.connect(dlg.accept)
        btns.rejected.connect(dlg.reject)
        vlay.addWidget(btns)

        if dlg.exec_() == QtWidgets.QDialog.Accepted:
            # اقرأ القيم
            duration = durEdit.text().strip()
            dist_lim = rangeEdit.text().strip()
            base_id  = baseEdit.text().strip()

            # تحقق من الإلزاميّات
            if not (duration and dist_lim):
                return  # يمكنك هنا عرض رسالة خطأ بدل الإرجاع الصامت

            # بناء الأمر
            cmd = f"MODE BASE TIME {duration} {dist_lim}"
            if base_id:
                cmd += f" {base_id}"

            # أرسله وكرر حتى تستلم ACK
            self.sendAndWaitUntilAck(cmd, timeout=1.0)


            # إذا لم نسمع ACK خلال timeout نعيد المحاولة تلقائياً

    # --------------------------------------------------------------------- #
    #                          واجهة الاستخدام                               #
    # --------------------------------------------------------------------- #
    def setSerial(self, ser):
        """يُمرَّر المقبض من MainWindow عند نجاح الاتصال."""
        self.ser = ser
        self._thread = Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    # --------------------------------------------------------------------- #
    #                       وظائف داخليـة (Private)                          #
    # --------------------------------------------------------------------- #
    def _append_line(self, text: str):
        """يُضيف سطراً إلى الترمينال مع تمرير سطر جديد."""
        self.terminal.appendPlainText(text)

    def _on_input_entered(self):
        """إرسال أمر عبر الـ COM عندما يضغط المستخدم Enter."""
        cmd = self.input.text().strip()
        if cmd and self.ser:
            sendCommand(self.ser, cmd)
            self._append_line(f"> {cmd}")
        self.input.clear()

    def _read_loop(self):
        while self._running:
            # ← إذا المنفذ غير مفتوح، اخرج من الخيط
            if not self.ser or not self.ser.is_open:
                break
            line = readData(self.ser)
            if not line:
                QtCore.QThread.msleep(40)
                continue

            line = line.strip()
            # 0) عرض أي سطر يصل في التيرمينال
            try:
                self.newLine.emit(line)
            except RuntimeError:
                break  # إذا حُذفت الصفحة أثناء التشغيل

            # 1) جملة GGA ⇒ إحداثيات + جودة fix + عدد الأقمار + HDOP + ارتفاع
            if line.startswith(("$GPGGA", "$GNGGA")):
                gga = parseGGA(line)
                if gga:
                    lat = self._to_decimal(gga["latitude"], gga["lat_direction"])
                    lon = self._to_decimal(gga["longitude"], gga["lon_direction"])
                    alt = float(gga["altitude"]) if gga["altitude"] else 0.0

                    # حدّث المعلومات
                    self._last_info.update({
                        'lat': lat,
                        'lon': lon,
                        'alt': alt,
                        'fix_quality': gga['fix_quality'],
                        'num_sat': gga['num_satellites'],
                        'hdop': gga['hdop'],
                    })
                    # بُثّ المعلومات المفصلة
                    self.locationDetailed.emit(self._last_info)

            # 2) جملة GSA ⇒ PDOP, VDOP
            elif "GSA" in line:
                gsa = parseGSA(line)
                if gsa:
                    self._last_info.update({
                        'pdop': gsa['pdop'],
                        'vdop': gsa['vdop'],
                    })
                    self.locationDetailed.emit(self._last_info)

            # 3) جملة RMC (إن أردت معالجتها)
            elif RMC_RE.match(line):
                # هنا لا نعدل self._last_info لأن RMC لا يصدر ارتفاعاً
                pass

            QtCore.QThread.msleep(40)


    def _update_coordinates(self, lat, lon, alt):
        """
        يحدِّث شريط الإحداثيات ويبعث (lat, lon, alt).
        هذا هو التعريف الذي اتصلت به في _read_loop.
        """
        if lat is None or lon is None:
            return

        # عرض الإحداثيات مع الارتفاع
        self.coordLabel.setText(
            f"Lat: {lat:.6f}  |  Lon: {lon:.6f}  |  Alt: {alt} m"
        )

        # تأكد من تحويل alt إلى float صالح
        try:
            alt_f = float(alt)
        except (TypeError, ValueError):
            alt_f = 0.0

        # بث الإشارة الثلاثية
        self.locationChanged.emit(lat, lon, alt_f)



    @staticmethod
    def _to_decimal(coord: str, direction: str):
        """تحويل إحداثى NMEA إلى درجة عشرية مع تحقّق من القيم."""
        if not coord:                           # فارغ أو None
            return None
        deg_len = 2 if direction in ("N", "S") else 3
        try:
            degrees = int(coord[:deg_len])
            minutes = float(coord[deg_len:])
        except ValueError:
            return None                         # قيمة غير صالحة
        decimal = degrees + minutes / 60
        if direction in ("S", "W"):
            decimal = -decimal
        return decimal

    
    def closeEvent(self, event):
        """يُستدعى عندما تُغلق الصفحة داخل الـ MDI."""
        self._running = False                       # أوقف الحلقة
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1)            # انتظر الخيط لحظة
        super().closeEvent(event)                   # أكمل الإغلاق


##############################################
# ConstellationCanvas: لوحة قطبية لرسم الأقمار
##############################################
class ConstellationCanvas(FigureCanvas):
    #ICON_ZOOM   = 0.045   # ضبّطه ليكون حجم الأيقونة ≈ الدائرة الرمادية
    #MARKER_SIZE = 300     # حجم الدائرة الرمادية (s فى scatter)

    FLAG_MAP = {          # مسار صورة العلم لكل نظام
        "GP": "flags/usa.png",       # GPS – علم الولايات المتّحدة
        "GL": "flags/russia.png",    # GLONASS
        "GA": "flags/eu.png",        # Galileo
        "BD": "flags/china.png",     # BeiDou
        "QZ": "flags/japan.png",     # QZSS
        "SB": "flags/sbas.png",      # SBAS شعار S
        "GI": "flags/china.png",     # NavIC / IRNSS
        "GB": "flags/china.png",     # بديل لبعض الأجهزة
        "GN": "flags/sbas.png",      # GN‑mixed SBAS
    }
    ICON_DIAMETER_PX = 18
    def __init__(self, parent=None, width=5, height=5, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi, tight_layout=True)
        self.ax = fig.add_subplot(111, polar=True)
        super().__init__(fig)
        self.setParent(parent)

        self._load_icons()       # ← حمِّل الأعلام مرّة واحدة
        self.configure_axes()

    # ----------------------------------------------------------
    #            المساعدة: تحميل أيقونات الأعلام
    # ----------------------------------------------------------
    def _load_icons(self):
        """
        نحمل الصورة ونحسب معامل التكبير (zoom) بحيث يصبح أكبر بُعد = ICON_DIAMETER_PX.
        نخزّن (الصورة, zoom) معًا لكل نظام.
        """
        self.icons = {}
        for talker, rel_path in self.FLAG_MAP.items():
            try:
                img = plt.imread(resource_path(rel_path))
                h, w = img.shape[:2]
                zoom = self.ICON_DIAMETER_PX / max(h, w)
                self.icons[talker] = (img, zoom)
            except FileNotFoundError:
                pass   # إذا لم نجد الصورة سيُرسَم مكانها دائرة


    def _get_icon(self, talker):
        return self.icons.get(talker[:2], (None, None))


    # ----------------------------------------------------------
    #                إعداد محاور القطبية
    # ----------------------------------------------------------
    def configure_axes(self):
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)
        self.ax.set_rlim(0, 90)
        self.ax.set_yticks([0, 30, 60, 90])
        self.ax.set_yticklabels(['90°', '60°', '30°', '0°'])
        self.ax.grid(True, linestyle='--', alpha=0.25)

    # ----------------------------------------------------------
    #      الدالة التى يستدعيها Созвездие كل ثانية
    # ----------------------------------------------------------
    def plot_satellites(self, sats):
        self.ax.clear()
        self.configure_axes()

        # ===== تصفيّة الأقمار: فقط PRN أرقام صالحة وإحداثيات كاملة =====
        clean_sats = [
            s for s in sats
            if s.get('prn') and s['prn'].isdigit()
            and s.get('elevation') is not None
            and s.get('azimuth')   is not None
        ]

        for sat in clean_sats:
            elev   = sat['elevation']
            azim   = sat['azimuth']
            prn    = sat['prn']
            talker = sat['system']

            if elev is None or azim is None:
                continue

            r  = 90 - elev
            th = math.radians(azim)
            x, y = th, r

            img, z = self._get_icon(talker)
            if img is not None:
                ab = AnnotationBbox(
                    OffsetImage(img, zoom=z, interpolation='bilinear'),
                    (x, y), frameon=False, xycoords='data'
                )
                self.ax.add_artist(ab)
            else:
                self.ax.scatter(x, y, s=300, c='lightgray', edgecolors='k')

            # اكتب رقم القمر بالأبيض مع ستروك أسود
            txt = self.ax.text(
                x, y, str(prn),
                color='white', fontsize=8, weight='bold',
                ha='center', va='center'
            )
            txt.set_path_effects([
                patheffects.Stroke(linewidth=1, foreground='black'),
                patheffects.Normal()
            ])

        self.draw()


##############################################
# Созвездие: صفحة رسم الأقمار الحقيقية
##############################################
class Созвездие(QtWidgets.QWidget):
    """
    صفحة رسم الأقمار – لا تفتح المنفذ بنفسها بل تنتظر setSerial()
    لتلقّي المقبض من MainWindow.
    """
    def __init__(self, parent=None):
        super().__init__(parent)

        self.ser = None          # يُملأ لاحقاً
        self.sats = []           # قائمة الأقمار النشطة

        # واجهة الرسم
        lay = QtWidgets.QVBoxLayout(self)
        self.canvas = ConstellationCanvas(self, 5, 5, 100)
        lay.addWidget(self.canvas)

        # مؤقّت للتحديث البصري
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(
            lambda: self.canvas.plot_satellites(self.sats)
        )
        self.timer.start(1000)

    # ------------------------------------------------------------------ #
    #              تُستدعى من MainWindow بعد نجاح الاتصال                 #
    # ------------------------------------------------------------------ #
    def setSerial(self, ser):
        if self.ser is not None:          # لا تفتح خيطاً ثانياً لنفس المقبض
            return
        self.ser = ser
        Thread(target=self._read_loop, daemon=True).start()

    # ------------------------------------------------------------------ #
    #                       حلقة القراءة الداخلية                        #
    # ------------------------------------------------------------------ #
    def _read_loop(self):
        buffer   = {}              # {'GP':[...جزئية...], ...}
        sats_map = {}              # {(sys,prn): {'elev':..,'azim':..,'t':timestamp}}

        while True:
            raw = readData(self.ser)
            talker, finished, sats = parseGSV(raw)
            if not sats:
                QtCore.QThread.msleep(25)
                continue

            # اجمع الشرائح
            buffer.setdefault(talker, []).extend(sats)
            if not finished:
                continue

            now = time.time()
            for sat in buffer[talker]:
                key = (sat['system'], sat['prn'])
                sat['last_seen'] = now            # ← طابع زمنى
                sats_map[key] = sat               # يكتب/يحدث القمر
            buffer[talker] = []                  # ابدأ سلسلة جديدة

            # احذف الأقمار الأقدم من 15 ثانية
            sats_map = {k: s for k, s in sats_map.items() if now - s['last_seen'] < 5}

            # صفّ القائمة المُرسَلة للرسم
            self.sats = list(sats_map.values())

            QtCore.QThread.msleep(40)







class Карта(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QtWidgets.QGridLayout(self)
        self.webView = QtWebEngineWidgets.QWebEngineView()
        self.layout.addWidget(self.webView, 0, 0)
        self.infoFrame = QtWidgets.QFrame()
        self.infoFrame.setStyleSheet("background-color: rgba(255, 255, 255, 0.7);"
                                     "border: 1px solid #ccc; border-radius: 5px;")
        infoLayout = QtWidgets.QVBoxLayout(self.infoFrame)
        infoLayout.setContentsMargins(8, 8, 8, 8)
        self.infoLabel = QtWidgets.QLabel("Долгота: ...\nШирота: ...\nВысота: ...\nUTC: ...")
        self.infoLabel.setStyleSheet("color: black;")
        infoLayout.addWidget(self.infoLabel)
        self.layout.addWidget(self.infoFrame, 0, 0, alignment=QtCore.Qt.AlignLeft | QtCore.Qt.AlignBottom)
        self.setMinimumSize(400, 300)
        html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset='utf-8'>
            <title>Yandex GNSS Map</title>
            <style>html,body,#map{{height:100%;margin:0}}</style>

            <!-- خرائط ياندكس -->
            <script src='https://api-maps.yandex.ru/2.1/?lang=en_RU'></script>
            <script>
            var map, marker;

            ymaps.ready(function () {{
                map = new ymaps.Map('map', {{
                center: [0,0],          // [lon, lat]
                zoom: 16
                }});

                marker = new ymaps.Placemark(
                    [0,0],
                    {{}},  // لا مُحتوى للبالون عند الإقلاع
                    {{ preset:'islands#redDotIcon' }}
                );
                map.geoObjects.add(marker);

            }});
            </script>
        </head>
        <body><div id='map'></div></body>
        </html>
        """
        self.webView.setHtml(html)

    
    @QtCore.pyqtSlot(float, float, float)
    def update_position(self, lat, lon, alt):
        # 1) حدِّد دقة العرض للنص (مثلاً 6–7 أرقام عشرية)
        self.infoLabel.setText(
            f"Широта: {lat:.7f}\n"
            f"Долгота: {lon:.7f}\n"
            f"Высота: {alt:.1f} м\n"
            f"UTC: {datetime.utcnow().strftime('%H:%M:%S')}"
        )

        # 2) حدد مستوى التكبير الذي تريده (مثلاً 18 أو 19)
        #    setCenter يمكن أن يأخذ الوسيط الثاني للـ zoom
        js = f"""
            marker.geometry.setCoordinates([{lat}, {lon}]);
            map.setCenter([{lat}, {lon}], 19);
            marker.balloon.open();
        """
        self.webView.page().runJavaScript(js)


    @QtCore.pyqtSlot(dict)
    def update_info(self, info):
        # استخرج القيم مع fallback
        lat  = info.get('lat')
        lon  = info.get('lon')
        alt  = info.get('alt')
        fq   = info.get('fix_quality')
        ns   = info.get('num_sat')
        pdop = info.get('pdop')
        hdop = info.get('hdop')
        vdop = info.get('vdop')

        # 1) نصوص بديلة إذا كانت None
        lat_text  = f"{lat:.7f}" if isinstance(lat, float) else "--"
        lon_text  = f"{lon:.7f}" if isinstance(lon, float) else "--"
        alt_text  = f"{alt:.2f} m" if isinstance(alt, (int, float)) else "--"
        fix_map   = {'0':'Invalid','1':'Single','2':'DGPS','4':'RTK','5':'RTK Float'}
        fix_text  = fix_map.get(fq, fq or "--")
        ns_text   = str(ns) if isinstance(ns, int) else "--"
        pdop_text = f"{pdop:.2f}" if isinstance(pdop, (int, float)) else "--"
        hdop_text = f"{hdop:.2f}" if isinstance(hdop, (int, float)) else "--"
        vdop_text = f"{vdop:.2f}" if isinstance(vdop, (int, float)) else "--"

        # 2) حساب دقة 2D/3D (مثال: UERE ثابت = 10m)
        uere = 10.0
        acc3d = f"{(pdop * uere):.2f} m" if isinstance(pdop, (int, float)) else "--"
        acc2d = f"{(hdop * uere):.2f} m" if isinstance(hdop, (int, float)) else "--"

        # 3) بنية النص للعرض
        txt = (
            f"Lon: {lon_text}\n"
            f"Lat: {lat_text}\n"
            f"Hgt: {alt_text}\n"
            f"FixType: {fix_text}\n"
            f"Sats: {ns_text}\n"
            f"PDOP: {pdop_text}\n"
            f"HDOP: {hdop_text}\n"
            f"VDOP: {vdop_text}\n"
            f"3D Acc: {acc3d}\n"
            f"2D Acc: {acc2d}"
        )
        self.infoLabel.setText(txt)

        # 4) إذا لدينا إحداثيات صحيحة، حدّد المركز
        if isinstance(lat, float) and isinstance(lon, float):
            # داخل update_position أو update_info
            js = f"""
                marker.geometry.setCoordinates([{lat}, {lon}]);
                map.setCenter([{lat}, {lon}], 19);
                marker.balloon.open();
            """
            self.webView.page().runJavaScript(js)






class Статус_отслеживания(QtWidgets.QWidget):
    FLAG_INFO = {
        "ALL": ("flags/all.png",    "ALL"),
        "GP" : ("flags/usa.png",    "GPS"),
        "GB" : ("flags/china.png",  "BDS"),
        "GL" : ("flags/russia.png", "GLO"),
        "GA" : ("flags/eu.png",     "GAL"),
        "QZ" : ("flags/japan.png",  "QZSS"),
        "SB" : ("flags/sbas.png",   "SBAS"),
        "BD" : ("flags/india.png",  "NAVIC GB"),
}


    def __init__(self, parent=None):
        super().__init__(parent)
        self.ser = None

        # 1) إنشاء شريط الأعلام
        flagsBar = QtWidgets.QHBoxLayout()
        flagsBar.setSpacing(6)
        self.flagBtns = {}
        for talker, (icon, label) in self.FLAG_INFO.items():
            btn = FlagButton(talker, icon, label)
            # تثبيت قيمة talker لكل زر بواسطة lambda افتراضية
            btn.clicked.connect(lambda _, t=talker: self._on_flag_clicked(t))
            self.flagBtns[talker] = btn
            flagsBar.addWidget(btn)
        flagsBar.addStretch()

        # 2) إضافة الـCanvas أسفل الأعلام
        self.canvas = TrackingCanvas(self)

        # 3) ضبط التخطيط العام
        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(flagsBar)
        layout.addWidget(self.canvas)

        # 4) اجعل الحالة الافتراضية ALL
        self.canvas.current_sys = "ALL"
        self.flagBtns["ALL"].setSelected(True)

        # 5) مؤقت التحديث
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.canvas.update_plot)
        self.timer.start(1000)

    def _on_flag_clicked(self, talker):
        # تغيير النظام الجاري
        self.canvas.current_sys = talker
        self.canvas.update_plot()
        # تحديث تمييز الزرّ المختار
        for tk, btn in self.flagBtns.items():
            btn.setSelected(tk == talker)


    # يُستدعى من MainWindow بعد الاتصال
    def setSerial(self, ser):
        if self.ser:  # لا نعيد فتح الحلقة
            return
        self.ser = ser
        Thread(target=self._read_loop, daemon=True).start()

    def _read_loop(self):
        build = {}
        while True:
            raw = readData(self.ser)
            talker, finished, sats = parseGSV(raw)
            if not sats:
                QtCore.QThread.msleep(20)
                continue

            build.setdefault(talker, []).extend(sats)
            if finished:
                # خزّن القائمة كاملة ثم ابدأ سلسلة جديدة
                self.canvas.sats_by_sys[talker] = build[talker]
                build[talker] = []
            QtCore.QThread.msleep(10)

class ModeConfigurationPage(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        lbl = QtWidgets.QLabel("Страница настройки режима", self)
        lbl.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(lbl)
        self.setMinimumSize(300, 200)

class MessageConfigurationPage(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        lbl = QtWidgets.QLabel("Страница конфигурации сообщений", self)
        lbl.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(lbl)
        self.setMinimumSize(300, 200)

class SerialPortConfigPage(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        lbl = QtWidgets.QLabel("Страница настройки последовательного порта", self)
        lbl.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(lbl)
        self.setMinimumSize(300, 200)

##############################################
# SettingsDialog: نافذة الإعدادات
##############################################
class SettingsDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowModality(QtCore.Qt.ApplicationModal)
        self.setWindowTitle("Настройки")
        self.resize(900, 600)
        mainLayout = QtWidgets.QHBoxLayout(self)
        mainLayout.setContentsMargins(8, 8, 8, 8)
        mainLayout.setSpacing(8)
        self.listItems = QtWidgets.QListWidget()
        self.listItems.addItem("Настройка режима")
        self.listItems.addItem("Конфигурация сообщений")
        self.listItems.addItem("Настройка последовательного порта")
        mainLayout.addWidget(self.listItems, 1)
        self.stack = QtWidgets.QStackedWidget()
        self.modePage = ModeConfigurationPage()
        self.messagePage = MessageConfigurationPage()
        self.serialPage = SerialPortConfigPage()
        self.stack.addWidget(self.modePage)
        self.stack.addWidget(self.messagePage)
        self.stack.addWidget(self.serialPage)
        mainLayout.addWidget(self.stack, 3)
        self.listItems.currentRowChanged.connect(self.stack.setCurrentIndex)
        self.listItems.setCurrentRow(0)
        self.stack.setCurrentIndex(0)

##############################################
# SplashScreen: شاشة البداية (الإقلاع)
##############################################
class SplashScreen(QtWidgets.QWidget):
    def __init__(self, gif_path, parent=None):
        super().__init__(parent)
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint | QtCore.Qt.SplashScreen)
        self.label = QtWidgets.QLabel(self)
        self.movie = QtGui.QMovie(resource_path(gif_path))
        self.label.setMovie(self.movie)
        self.movie.start()
        self.resize(self.movie.currentPixmap().size())

##############################################
# ConnectionsDialog: نافذة الاتصالات
##############################################

class ConnectionsDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowModality(QtCore.Qt.ApplicationModal)
        self.setWindowTitle("Соединения")
        self.setWindowFlags(self.windowFlags() & ~QtCore.Qt.WindowContextHelpButtonHint)
        mainLayout = QtWidgets.QVBoxLayout(self)
        groupSerial = QtWidgets.QGroupBox("Последовательный порт")
        groupSerialLayout = QtWidgets.QHBoxLayout(groupSerial)
        self.comboPorts = QtWidgets.QComboBox()
        self.comboPorts.addItems([f"COM{i}" for i in range(1, 5)])
        groupSerialLayout.addWidget(self.comboPorts)
        self.comboBaud = QtWidgets.QComboBox()
        self.comboBaud.addItems(["4800", "9600", "19200", "115200", "AUTO"])
        groupSerialLayout.addWidget(self.comboBaud)
        self.remarkSerial = QtWidgets.QLineEdit()
        self.remarkSerial.setPlaceholderText("Заметка")
        groupSerialLayout.addWidget(self.remarkSerial)
        mainLayout.addWidget(groupSerial)
        groupTCP = QtWidgets.QGroupBox("TCP/IP")
        groupTCPLayout = QtWidgets.QHBoxLayout(groupTCP)
        self.lineIP = QtWidgets.QLineEdit()
        self.lineIP.setPlaceholderText("IP-адрес")
        groupTCPLayout.addWidget(self.lineIP)
        self.linePort = QtWidgets.QLineEdit()
        self.linePort.setPlaceholderText("Порт")
        groupTCPLayout.addWidget(self.linePort)
        self.remarkTcp = QtWidgets.QLineEdit()
        self.remarkTcp.setPlaceholderText("Заметка")
        groupTCPLayout.addWidget(self.remarkTcp)
        mainLayout.addWidget(groupTCP)
        buttonsLayout = QtWidgets.QHBoxLayout()
        self.btnOk = QtWidgets.QPushButton("ОК")
        self.btnCancel = QtWidgets.QPushButton("Отмена")
        buttonsLayout.addStretch()
        buttonsLayout.addWidget(self.btnOk)
        buttonsLayout.addWidget(self.btnCancel)
        mainLayout.addLayout(buttonsLayout)
        self.btnOk.clicked.connect(self.accept)
        self.btnCancel.clicked.connect(self.reject)
        self.resize(500, 200)
        qtRect = QtWidgets.QApplication.desktop().availableGeometry().center()
        dlgRect = self.rect().center()
        self.move(qtRect - dlgRect)

    def getConnectionData(self):
        return {
            "serial_port": self.comboPorts.currentText(),
            "baudrate": self.comboBaud.currentText(),
            "remarkSerial": self.remarkSerial.text(),
            "ip_address": self.lineIP.text(),
            "ip_port": self.linePort.text(),
            "remarkTcp": self.remarkTcp.text()
        }

##############################################
# CustomMdiSubWindow: نافذة داخلية قابلة للتعديل
##############################################
class CustomMdiSubWindow(QtWidgets.QMdiSubWindow):
    def __init__(self, contentWidget, title=None, parent=None):
        super().__init__(parent)
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint | QtCore.Qt.Window)
        self._isMaximized = False
        self._resize_margin = 10
        self._is_resizing = False
        self._resize_direction = None
        self._start_pos = None
        self._start_geometry = None

        self.titleBar = QtWidgets.QWidget()
        self.titleBar.setFixedHeight(30)
        self.titleBar.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.titleBar.setStyleSheet("background-color: #444; color: white;")
        titleLayout = QtWidgets.QHBoxLayout(self.titleBar)
        titleLayout.setContentsMargins(2, 2, 2, 2)
        titleLayout.setSpacing(2)
        self.lblTitle = QtWidgets.QLabel(title if title is not None else "", self.titleBar)
        titleLayout.addWidget(self.lblTitle)
        titleLayout.addStretch()
        self.btnMin = QtWidgets.QPushButton("-", self.titleBar)
        self.btnMin.setFixedSize(20, 20)
        self.btnMin.setStyleSheet("background-color: #666; color: white; border: none;")
        self.btnMin.clicked.connect(self.showMinimized)
        titleLayout.addWidget(self.btnMin)
        self.btnMax = QtWidgets.QPushButton("□", self.titleBar)
        self.btnMax.setFixedSize(20, 20)
        self.btnMax.setStyleSheet("background-color: #666; color: white; border: none;")
        self.btnMax.clicked.connect(self.toggleMaxRestore)
        titleLayout.addWidget(self.btnMax)
        self.btnClose = QtWidgets.QPushButton("×", self.titleBar)
        self.btnClose.setFixedSize(20, 20)
        self.btnClose.setStyleSheet("background-color: #666; color: white; border: none;")
        self.btnClose.clicked.connect(self.close)
        titleLayout.addWidget(self.btnClose)
        self.mainWidget = QtWidgets.QWidget()
        self.layout = QtWidgets.QVBoxLayout(self.mainWidget)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(0)
        self.layout.addWidget(self.titleBar)
        self.layout.addWidget(contentWidget)
        self.setWidget(self.mainWidget)
        self.oldPos = None
        self.titleBar.mousePressEvent = self.titleBarMousePress
        self.titleBar.mouseMoveEvent = self.titleBarMouseMove
        self.titleBar.mouseReleaseEvent = self.titleBarMouseRelease

    def titleBarMousePress(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.oldPos = event.globalPos()
        event.accept()

    def titleBarMouseMove(self, event):
        if self.oldPos and not self._is_resizing:
            delta = event.globalPos() - self.oldPos
            self.move(self.x() + delta.x(), self.y() + delta.y())
            self.oldPos = event.globalPos()
        event.accept()

    def titleBarMouseRelease(self, event):
        self.oldPos = None
        event.accept()

    def toggleMaxRestore(self):
        if self._isMaximized:
            self.showNormal()
            self._isMaximized = False
            self.btnMax.setText("□")
        else:
            self.showMaximized()
            self._isMaximized = True
            self.btnMax.setText("❐")

    def event(self, e):
        if e.type() == QtCore.QEvent.MouseMove:
            if self._is_resizing:
                self.resizeWindow(e.globalPos())
                return True
            else:
                self.updateCursor(e.globalPos())
        elif e.type() == QtCore.QEvent.MouseButtonPress:
            if e.button() == QtCore.Qt.LeftButton:
                direction = self.getResizeDirection(e.globalPos())
                if direction:
                    self._is_resizing = True
                    self._resize_direction = direction
                    self._start_pos = e.globalPos()
                    self._start_geometry = self.geometry()
                    return True
        elif e.type() == QtCore.QEvent.MouseButtonRelease:
            if e.button() == QtCore.Qt.LeftButton and self._is_resizing:
                self._is_resizing = False
                self._resize_direction = None
                self.unsetCursor()
                return True
        return super().event(e)

    def getResizeDirection(self, globalPos):
        margin = self._resize_margin
        rect = self.frameGeometry()
        direction = ""
        if abs(globalPos.x() - rect.left()) <= margin:
            direction += "left"
        elif abs(globalPos.x() - rect.right()) <= margin:
            direction += "right"
        if abs(globalPos.y() - rect.top()) <= margin:
            direction = "top" + direction
        elif abs(globalPos.y() - rect.bottom()) <= margin:
            direction = "bottom" + direction
        return direction if direction != "" else None

    def updateCursor(self, globalPos):
        direction = self.getResizeDirection(globalPos)
        if direction:
            if direction in ["left", "right"]:
                self.setCursor(QtCore.Qt.SizeHorCursor)
            elif direction in ["top", "bottom"]:
                self.setCursor(QtCore.Qt.SizeVerCursor)
            elif direction in ["topleft", "bottomright"]:
                self.setCursor(QtCore.Qt.SizeFDiagCursor)
            elif direction in ["topright", "bottomleft"]:
                self.setCursor(QtCore.Qt.SizeBDiagCursor)
            else:
                self.unsetCursor()
        else:
            self.unsetCursor()

    def resizeWindow(self, globalPos):
        delta = globalPos - self._start_pos
        rect = QtCore.QRect(self._start_geometry)
        direction = self._resize_direction
        if "right" in direction:
            rect.setRight(rect.right() + delta.x())
        if "left" in direction:
            rect.setLeft(rect.left() + delta.x())
        if "bottom" in direction:
            rect.setBottom(rect.bottom() + delta.y())
        if "top" in direction:
            rect.setTop(rect.top() + delta.y())
        minWidth = self.minimumWidth() if self.minimumWidth() > 0 else 100
        minHeight = self.minimumHeight() if self.minimumHeight() > 0 else 100
        if rect.width() < minWidth:
            if "left" in direction:
                rect.setLeft(rect.right() - minWidth)
            else:
                rect.setRight(rect.left() + minWidth)
        if rect.height() < minHeight:
            if "top" in direction:
                rect.setTop(rect.bottom() - minHeight)
            else:
                rect.setBottom(rect.top() + minHeight)
        self.setGeometry(rect)

##############################################
# MainWindow: النافذة الرئيسية مع شريط رأس مخصص
##############################################
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.subwindows = {}          # {PageClass: QMdiSubWindow}
        # بلا إطار
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.resize(1200, 800)
        self.isCollapsed = True
        self.sidebarWidthCollapsed = 90
        self.sidebarWidthExpanded  = 130
        self.oldPos = None
        self.currentSerial = None
        self.initUI()

    def populateSerialPorts(self):
        """يملأ comboPorts بالمنافذ الموجودة حاليًّا فقط."""
        self.comboPorts.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.comboPorts.addItems(ports)


    def initUI(self):
        # —— Header setup —— #
        self.header = QtWidgets.QWidget()
        self.header.setFixedHeight(60)
        hl = QtWidgets.QHBoxLayout(self.header)
        hl.setContentsMargins(10,0,10,0)
        hl.setSpacing(10)

        # شعار
        self.logoLabel = QtWidgets.QLabel()
        pix = QtGui.QPixmap(resource_path("images/company_logo.gif"))
        self.logoLabel.setPixmap(pix.scaled(100,100,QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        hl.addWidget(self.logoLabel)

        # عنوان فارغ
        title = QtWidgets.QLabel("")
        title.setStyleSheet("font-size:16px;font-weight:bold;color:black;")
        hl.addWidget(title)
        hl.addStretch(1)

        # أزرار التحكم
        self.refreshButton = QtWidgets.QPushButton("↻")
        self.refreshButton.setFixedSize(30, 30)
        self.refreshButton.clicked.connect(self.refreshSubwindows)
        hl.addWidget(self.refreshButton)
        self.minButton   = QtWidgets.QPushButton("-");  self.minButton.setFixedSize(30,30)
        self.minButton.clicked.connect(self.showMinimized); hl.addWidget(self.minButton)
        self.maxButton   = QtWidgets.QPushButton("□");  self.maxButton.setFixedSize(30,30)
        self.maxButton.clicked.connect(self.toggleMaxRestore); hl.addWidget(self.maxButton)
        self.closeButton = QtWidgets.QPushButton("×");  self.closeButton.setFixedSize(30,30)
        self.closeButton.clicked.connect(self.close); hl.addWidget(self.closeButton)

        # سحب النافذة
        self.header.mousePressEvent = self.mousePressEventHeader
        self.header.mouseMoveEvent  = self.mouseMoveEventHeader

        # —— Sidebar & MDI —— #
        central = QtWidgets.QWidget()
        cl = QtWidgets.QHBoxLayout(central)
        cl.setContentsMargins(0,0,0,0); cl.setSpacing(0)

        # الشريط الجانبي
        self.sidebar = QtWidgets.QWidget(); self.sidebar.setObjectName("sidebar")
        self.sidebar.setFixedWidth(self.sidebarWidthCollapsed)
        sl = QtWidgets.QVBoxLayout(self.sidebar)
        sl.setContentsMargins(0,0,0,0); sl.setSpacing(0)

        # زر طي/توسع
        self.toggleButton = QtWidgets.QToolButton()
        self.toggleButton.setObjectName("toggleButton")
        self.toggleButton.setIcon(QtGui.QIcon(resource_path("images/sidebar-left.png")))
        self.toggleButton.setIconSize(QtCore.QSize(24,24))
        self.toggleButton.setToolButtonStyle(QtCore.Qt.ToolButtonIconOnly)
        self.toggleButton.clicked.connect(self.toggleSidebar)
        tsl = QtWidgets.QHBoxLayout(); tsl.setContentsMargins(5,5,5,5)
        tsl.addWidget(self.toggleButton, alignment=QtCore.Qt.AlignLeft)
        twidget = QtWidgets.QWidget(); twidget.setLayout(tsl)
        sl.addWidget(twidget); sl.addStretch(1)

        # قائمة الأزرار
        self.menuItems = [
            ("Поток данных","images/data_stream.png",  Поток_данных),
            ("Созвездие",  "images/constellation.png", Созвездие),
            ("Статус отслеживания","images/tracking_status.png",Статус_отслеживания),
            ("Карта",      "images/map.png",           Карта),
        ]
        self.buttons = []
        ml = QtWidgets.QVBoxLayout(); ml.setSpacing(5); ml.setAlignment(QtCore.Qt.AlignHCenter)
        for text, icon, cls in self.menuItems:
            btn = IconTextButton(icon, text)
            if cls:   # إذا كان هناك صنف صفحة
                btn.clicked.connect(partial(self.openSubWindow, cls))
            elif text == "Соединения":
                btn.clicked.connect(self.showConnectionsDialog)
            elif text == "Настройки":
                btn.clicked.connect(self.showSettingsDialog)
            # لا تغيّر ترتيب إضافة الأزرار للمكدّس ml:
            ml.addWidget(btn)
            self.buttons.append((btn, text))
        bwidget = QtWidgets.QWidget(); bwidget.setLayout(ml)
        sl.addWidget(bwidget); sl.addStretch(1)

        # منطقة MDI
        self.mdiArea = QtWidgets.QMdiArea()
        self.mdiArea.setStyleSheet("background-color:white;")
        cl.addWidget(self.sidebar, 0)
        cl.addWidget(self.mdiArea, 1)

        # جمع الواجهة
        mainL = QtWidgets.QVBoxLayout()
        mainL.setContentsMargins(0,0,0,0); mainL.setSpacing(0)
        mainL.addWidget(self.header)
        mainL.addWidget(central, stretch=1)
        container = QtWidgets.QWidget(); container.setLayout(mainL)
        self.setCentralWidget(container)

        # StatusBar
        self.statusbar = QtWidgets.QStatusBar(); self.setStatusBar(self.statusbar)
        self.comboPorts = QtWidgets.QComboBox()
        self.comboBaud  = QtWidgets.QComboBox(); self.comboBaud.addItems(["9600","115200"])
        self.btnConnect = QtWidgets.QPushButton("Подключиться")
        self.statusbar.addPermanentWidget(QtWidgets.QLabel("Порт:"))
        self.statusbar.addPermanentWidget(self.comboPorts)
        self.statusbar.addPermanentWidget(QtWidgets.QLabel("Скорость:"))
        self.statusbar.addPermanentWidget(self.comboBaud)
        self.statusbar.addPermanentWidget(self.btnConnect)
        self.arrangeBtn = QtWidgets.QPushButton("Расположить окна")
        self.statusbar.addPermanentWidget(self.arrangeBtn)
        self.arrangeBtn.clicked.connect(self.arrangeWindows)
        self.populateSerialPorts()      # ← بدلاً من ترقيم ثابت
        self.btnConnect.clicked.connect(self.connectPort)

    def mousePressEventHeader(self, e):
        if e.button()==QtCore.Qt.LeftButton: self.oldPos = e.globalPos()
        e.accept()

    def mouseMoveEventHeader(self, e):
        if self.oldPos:
            delta = e.globalPos()-self.oldPos
            self.move(self.x()+delta.x(), self.y()+delta.y())
            self.oldPos = e.globalPos()
        e.accept()

    def toggleMaxRestore(self):
        if self.isMaximized(): self.showNormal(); self.maxButton.setText("□")
        else: self.showMaximized(); self.maxButton.setText("❐")

    def openSubWindow(self, pageClass, activate=True):
        """
        يفتح النافذة إذا لم تكن موجودة، أو يُفعّل القائمة الحالية فقط.
        pageClass: الصنف (مثلاً Поток_данных) وليس المثيل.
        """
        # 1) هل هذه الصفحة مفتوحة سابقاً؟
        sub = self.subwindows.get(pageClass)
        if sub and not sub.isHidden():
            if activate:
                self.mdiArea.setActiveSubWindow(sub)
            return sub

        # 2) لم تُفتح بعد أو أغلقت → أنشئها
        pageWidget = pageClass()
        if self.currentSerial and hasattr(pageWidget, "setSerial"):
            pageWidget.setSerial(self.currentSerial)

        sub = CustomMdiSubWindow(pageWidget, title=pageClass.__name__)
        sub.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.mdiArea.addSubWindow(sub)
        self.subwindows[pageClass] = sub

        # عندما يُدمّر الـ sub أزل المفتاح حتى يمكن فتحه لاحقاً
        sub.destroyed.connect(lambda _, c=pageClass: self.subwindows.pop(c, None))

        # وصل إشارات الخرائط/الإحداثيات
        if isinstance(pageWidget, Поток_данных):
            self.dataPage = pageWidget
            if hasattr(self, "Карта"):
                # الآن سيُربط بثلاثي المتغيّرات
                pageWidget.locationChanged.connect(self.Карта.update_position)

        if isinstance(pageWidget, Карта):
            self.Карта = pageWidget
            if hasattr(self, "dataPage"):
                self.dataPage.locationChanged.connect(pageWidget.update_position)

                # إذا هذا هو Поток_данных وسبق وأنشأنا Карта
        if isinstance(pageWidget, Поток_данных) and hasattr(self, 'Карта'):
            # احتفظ بالإشارة إليه
            self.dataPage = pageWidget
            # اربط السIGNAL بالدالة update_info في Карта
            pageWidget.locationDetailed.connect(self.Карта.update_info)

        # إذا هذا هو Карта وسبق وأنشأنا Поток_данных
        if isinstance(pageWidget, Карта) and hasattr(self, 'dataPage'):
            self.Карта = pageWidget
            self.dataPage.locationDetailed.connect(pageWidget.update_info)

        sub.show()
        return sub



    def showConnectionsDialog(self):
        dlg = ConnectionsDialog(self)
        center = QtWidgets.QApplication.desktop().availableGeometry().center()
        dlg.move(center - dlg.rect().center())
        if dlg.exec_()==QtWidgets.QDialog.Accepted:
            data = dlg.getConnectionData(); print("Данные:", data)

    def showSettingsDialog(self):
        dlg = SettingsDialog(self)
        center = QtWidgets.QApplication.desktop().availableGeometry().center()
        dlg.move(center - dlg.rect().center()); dlg.exec_()

    def arrangeWindows(self):
        W, H = self.mdiArea.width(), self.mdiArea.height()
        mapW   = int(W * 0.30); leftW = W - mapW
        upH    = int(H * 0.50); lowH  = H - upH
        constW = int(leftW * 0.45)

        # 1) Карта أولاً
        self._resizeSub(self.openSubWindow(Карта, activate=False),
                        leftW, 0, mapW, H)
        # 2) بقيّة الصفحات
        self._resizeSub(self.openSubWindow(Созвездие, activate=False),
                        0, 0, constW, upH)
        self._resizeSub(self.openSubWindow(Поток_данных, activate=False),
                        constW, 0, leftW - constW, upH)
        self._resizeSub(self.openSubWindow(Статус_отслеживания, activate=False),
                        0, upH, leftW, lowH)



    def refreshSubwindows(self):
        """يغلق كل النوافذ ثم يعيد إنشاءها وترتيبها."""
        # 1) أغلق جميع النوافذ الداخلية
        for sub in self.mdiArea.subWindowList()[:]:
            sub.close()

        # 2) احذف المراجع إلى صفحات قديمة إن وُجدت
        for attr in ("dataPage", "Карта"):
            if hasattr(self, attr):
                delattr(self, attr)

        # 3) أعد إنشـاء الصفحات فى الإطار الرئيسى
        QtCore.QTimer.singleShot(0, self.arrangeWindows)



    def _resizeSub(self, sub, x, y, w, h):
        """مساعد لضبط حجم وموقع الـ QMdiSubWindow"""
        if sub:
            sub.setGeometry(x, y, w, h)
            sub.show()


    def openAndResizeSubWindow(self, cls, name, x, y, w, h):
        # ابحث إن كانت النافذة موجودة
        for sub in self.mdiArea.subWindowList():
            if sub.windowTitle() == name:
                sub.setGeometry(x, y, w, h)
                sub.show()
                return

        # إنشاء نافذة جديدة
        widget = cls()
        if self.currentSerial and hasattr(widget, "setSerial"):
            widget.setSerial(self.currentSerial)

        sub = CustomMdiSubWindow(widget, title=name)
        sub.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.mdiArea.addSubWindow(sub)
        sub.setGeometry(x, y, w, h)
        sub.show()

    def toggleSidebar(self):
        if self.isCollapsed:
            self.sidebar.setFixedWidth(self.sidebarWidthExpanded)
            self.toggleButton.setIcon(QtGui.QIcon(resource_path("images/sidebar-left.png")))
            for btn,_ in self.buttons: btn.setExpanded(True)
        else:
            self.sidebar.setFixedWidth(self.sidebarWidthCollapsed)
            self.toggleButton.setIcon(QtGui.QIcon(resource_path("images/sidebar-right.png")))
            for btn,_ in self.buttons: btn.setExpanded(False)
        self.isCollapsed = not self.isCollapsed

    def connectPort(self):
        # -- إذا كان المستخدم متصلاً حالياً → فصل الاتصال بأمان --
        if self.currentSerial:
            # 1) أوقف خيوط القراءة في الصفحات المفتوحة التي تستخدم _running
            for sub in self.mdiArea.subWindowList():
                page = sub.widget()
                if hasattr(page, "_running"):
                    page._running = False

            # 2) أغلق المقبض بعد توقف الخيوط
            closeConnection(self.currentSerial)
            self.currentSerial = None

            # 3) حدّث حالة الزرّ والإشعار
            self.statusbar.showMessage("Соединение закрыто", 5000)
            self.btnConnect.setText("Подключиться")
            return

        # -- غير متصل → حاول إنشاء الاتصال --
        port = self.comboPorts.currentText()
        baud = int(self.comboBaud.currentText())

        ser = openConnection(port, baud)
        if not ser:
            self.statusbar.showMessage("Ошибка подключения", 5000)
            return

        # -- نجح الاتصال --
        self.currentSerial = ser
        self.statusbar.showMessage(f"Подключено на {port} @ {baud}", 5000)
        self.btnConnect.setText("Отключиться")

        #  مرّر المقبض لكل الصفحات المفتوحة
        for sub in self.mdiArea.subWindowList():
            page = sub.widget()
            if hasattr(page, "setSerial"):
                page.setSerial(ser)

##############################################
# التطبيق الرئيسي
##############################################
import os
import sys
import tempfile
import requests
import subprocess

VERSION = "1.0.0"
VERSION_URL = "https://raw.githubusercontent.com/eemaraa/myproject1/refs/heads/main/version.txt"
EXE_URL     = "https://github.com/eemaraa/myproject1/releases/latest/download/SelkhozRisheniya.exe"

def auto_update():

    
    try:
        # 1. طلب رقم النسخة الجديدة من GitHub
        r = requests.get(VERSION_URL, timeout=5)
        r.raise_for_status()
        latest = r.text.strip()

        # 2. لو فيه إصدار أحدث
        if latest != VERSION:
            print(f"⏬ There is a new update: {latest}")

            # 3. تحميل النسخة الجديدة إلى ملف مؤقت
            temp_path = os.path.join(tempfile.gettempdir(), "new.exe")
            r2 = requests.get(EXE_URL, stream=True, timeout=10)
            with open(temp_path, "wb") as f:
                for chunk in r2.iter_content(1024 * 1024):
                    f.write(chunk)

            print("✅ Updated")

            # 4. إنشاء سكربت bat لاستبدال النسخة الحالية بعد الإغلاق
            bat_path = os.path.join(tempfile.gettempdir(), "update.bat")
            current_exe = sys.argv[0]

            script = f"""
            @echo off
            timeout /t 2 >nul
            move /Y "{temp_path}" "{current_exe}"
            start "" "{current_exe}"
            """

            with open(bat_path, "w", encoding="utf-8") as f:
                f.write(script)

            # 5. تشغيل السكربت و إنهاء النسخة القديمة
            subprocess.Popen(['cmd', '/c', bat_path], shell=True)
            sys.exit(0)

    except Exception as e:
        print("❌ فشل التحقق من التحديث:", e)


def main():
    auto_update()  # ← إضافة هذه السطر في البداية

    app = QtWidgets.QApplication(sys.argv)

    # باقي الكود كما هو
    app.setStyleSheet("""
    QMainWindow {
        background-color: white;
    }
    QWidget#sidebar {
        background-color: #90EE90;
    }
    QToolButton#toggleButton {
        background-color: transparent;
        color: black;
        border: none;
        margin: 5px;
    }
    QLabel {
        color: black;
    }
    QStatusBar {
        background-color: #DEF2F1;
        font-size: 12px;
    }
    """)

    splash = SplashScreen(os.path.join("images", "logo.gif"))
    splash.show()
    app.processEvents()

    window = MainWindow()
    QtCore.QTimer.singleShot(8000, splash.close)
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
