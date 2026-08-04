"""
╔══════════════════════════════════════════════════════╗
║        PATIENT MONITOR VIEWER v2.0                   ║
║                                                      ║
║  Program ini berjalan di LAPTOP/PC                   ║
║  Bukan di ESP32, bukan di Google                     ║
║  Tugasnya: ambil data dari Google Sheets             ║
║            lalu tampilkan sebagai grafik sinyal      ║
╚══════════════════════════════════════════════════════╝
"""

# ── LIBRARY yang dipakai ──────────────────────────────
import tkinter as tk
# tkinter = library bawaan Python untuk bikin jendela, tombol, label dll
from tkinter import ttk, messagebox, filedialog
# ttk = widget yang lebih modern (combobox, progressbar)
# messagebox = popup pesan error/info
# filedialog = dialog "Simpan File" saat export

import gspread
# gspread = library Python khusus untuk baca/tulis Google Sheets
# harus diinstall dulu: pip install gspread

from google.oauth2.service_account import Credentials
# Untuk login ke Google pakai file credentials.json
# credentials.json = file kunci akses dari Google Cloud Console

import matplotlib.pyplot as plt
# matplotlib = library untuk bikin grafik/chart di Python
import matplotlib.ticker as ticker
# ticker = atur format angka di sumbu grafik
import matplotlib.patches as mpatches
# mpatches = bentuk-bentuk (kotak, lingkaran) di grafik
import matplotlib.dates as mdates
# mdates = format tanggal/waktu di sumbu X grafik

import numpy as np
# numpy = library matematika: array, persentil, cari nilai minimum dll

from datetime import datetime, timedelta
# datetime = tipe data tanggal+waktu
# timedelta = selisih waktu (misal: +1000 milidetik)

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
# FigureCanvasTkAgg = tempelkan grafik matplotlib ke dalam jendela tkinter
# NavigationToolbar2Tk = toolbar zoom/pan di bawah grafik

from matplotlib.figure import Figure
# Figure = objek kanvas tempat grafik digambar

import pandas as pd
# pandas = library tabel data di Python
# seperti Excel tapi dikontrol lewat kode
# DataFrame = tabel dengan baris dan kolom

import threading
# threading = jalankan proses di background
# supaya UI tidak freeze saat ambil data dari internet

import os
# os = akses file dan folder di sistem


# ============================================================
# KONFIGURASI
# ============================================================
SPREADSHEET_ID = "1a3goaBrfqC0RdgkvCRScDkN4Y3i06z4pQPvCq6nNolQ"
# ID spreadsheet Google Sheets yang mau dibaca
# Diambil dari URL spreadsheet:
# docs.google.com/spreadsheets/d/[ID INI]/edit
# Harus sama persis dengan ID di Apps Script

CREDENTIALS_FILE = "credentials.json"
# Nama file kunci login ke Google API
# Cara dapat: Google Cloud Console → Service Account → Download JSON
# Letakkan di folder yang sama dengan main.py ini

SCOPES = [
    "https://www.googleapis.com/auth/spreadsheets",
    "https://www.googleapis.com/auth/drive"
]
# SCOPES = daftar izin yang diminta ke Google
# Seperti: "saya minta izin untuk baca/tulis Sheets dan Drive"
# Tanpa ini Google tidak akan mengizinkan akses


# ============================================================
# WARNA TAMPILAN APLIKASI
# ============================================================
BG_DARK    = "#0a0a0a"  # Latar belakang utama (hitam pekat)
BG_PANEL   = "#111111"  # Panel sidebar (abu sangat gelap)
BG_CARD    = "#1a1a1a"  # Kotak kecil di dalam panel
ACCENT     = "#7c3aed"  # Ungu → warna tombol utama
ACCENT2    = "#06b6d4"  # Cyan → warna teks saat hover grafik
TEXT_WHITE = "#ffffff"  # Teks putih
TEXT_GRAY  = "#888888"  # Teks abu-abu untuk keterangan
BTN_GREEN  = "#10b981"  # Tombol hijau
BTN_RED    = "#ef4444"  # Tombol merah
BTN_BLUE   = "#3b82f6"  # Tombol biru
BTN_ORANGE = "#f59e0b"  # Tombol oranye

SIGNAL_COLORS = {
    "ECG Lead I":   "#00ff88",  # Hijau terang
    "ECG Lead II":  "#00cfff",  # Biru muda
    "ECG Lead III": "#ff6b6b",  # Merah muda
    "SpO₂ Wave":    "#ffd93d",  # Kuning
    "Resp. Wave":   "#c77dff"   # Ungu muda
}
# Tiap sinyal punya warna berbeda supaya mudah dibedakan di grafik

SIGNAL_COLS = {
    "ECG Lead I":   "ECG_Lead1",   # Nama kolom di Google Sheets
    "ECG Lead II":  "ECG_Lead2",
    "ECG Lead III": "ECG_Lead3",
    "SpO₂ Wave":    "SpO2_Wave",
    "Resp. Wave":   "Resp_Wave",
}
# Mapping: nama sinyal di UI → nama kolom di Sheets
# Contoh: "ECG Lead I" di layar = kolom "ECG_Lead1" di spreadsheet

ACTIVE_THRESHOLD = 1.0
# Batas untuk menentukan sinyal aktif atau tidak
# Standar deviasi sinyal <= 1.0 → dianggap flat (tidak ada data)
# Standar deviasi = ukuran seberapa bervariasi nilai sinyal
# Kalau semua nilai sama/0 → std = 0 → sinyal flat → disembunyikan


# ============================================================
# FUNGSI LOGIN KE GOOGLE SHEETS
# ============================================================
def connect_sheets():
    try:
        creds = Credentials.from_service_account_file(CREDENTIALS_FILE, scopes=SCOPES)
        # Baca file credentials.json → buat objek login
        # Ibarat: buka KTP untuk verifikasi identitas ke Google

        client = gspread.authorize(creds)
        # Login ke Google API pakai credentials tadi
        # Hasilnya: client = akun yang sudah terverifikasi dan bisa akses Sheets

        return client.open_by_key(SPREADSHEET_ID)
        # Buka spreadsheet menggunakan ID
        # Return: objek spreadsheet → bisa dibaca sheet-sheet di dalamnya

    except FileNotFoundError:
        messagebox.showerror("Error",
            f"File '{CREDENTIALS_FILE}' not found!\n"
            "Please place credentials.json in the same folder.")
        return None
        # credentials.json tidak ditemukan → tampilkan popup error → return None

    except Exception as e:
        messagebox.showerror("Connection Error",
            f"Failed to connect to Google Sheets:\n{str(e)}")
        return None
        # Error lain (internet mati, ID salah, izin kurang) → popup error → return None


# ============================================================
# FUNGSI UBAH TEKS TANGGAL → OBJEK DATETIME
# Contoh input : "27/04/2026 2:11:40"
# Contoh output: datetime(2026, 4, 27, 2, 11, 40)
# ============================================================
def parse_start_time(s):
    s = str(s).strip()
    # Pastikan tipe string, hapus spasi di tepi kiri/kanan

    for fmt in ("%d/%m/%Y %H:%M:%S", "%d/%m/%Y %H:%M",
                "%m/%d/%Y %H:%M:%S", "%m/%d/%Y %H:%M"):
        # Coba 4 format tanggal yang berbeda satu per satu
        # %d/%m/%Y = 27/04/2026 (format Indonesia)
        # %m/%d/%Y = 04/27/2026 (format Amerika)
        # %H:%M:%S = 10:15:30 (ada detik)
        # %H:%M    = 10:15    (tanpa detik)
        try:
            return datetime.strptime(s, fmt)
            # Kalau format cocok → return objek datetime
        except ValueError:
            continue
            # Format tidak cocok → coba format berikutnya

    return None
    # Semua format dicoba dan gagal → return None


# ============================================================
# KELAS UTAMA APLIKASI
# Semua tampilan dan logika ada di dalam kelas ini
# ============================================================
class PatientMonitorApp:
    def __init__(self, root):
        # Fungsi ini dijalankan pertama kali saat objek dibuat
        self.root = root
        self.root.title("Data Logger Patient Monitor v2.0")
        # Judul di title bar jendela

        self.root.geometry("1400x900")
        # Ukuran jendela saat pertama dibuka: lebar 1400px, tinggi 900px

        self.root.configure(bg=BG_DARK)
        # Warna latar belakang jendela utama

        self.root.minsize(1000, 700)
        # Ukuran minimum jendela: tidak bisa dikecilkan kurang dari ini

        self.spreadsheet  = None
        # Akan diisi dengan objek spreadsheet setelah login berhasil

        self.recordings   = []
        # List rekaman yang sudah DONE, diambil dari sheet "Recordings"

        self.current_data = None
        # DataFrame berisi data sensor rekaman yang sedang ditampilkan

        self.current_rec  = None
        # Dictionary info rekaman yang sedang ditampilkan (ID, waktu, dll)

        self.axes         = []
        # List objek subplot/axes grafik yang sedang tampil

        self._linked      = True
        # True = semua grafik scroll/zoom bersamaan (linked X axis)

        self._setup_style()   # Atur tampilan widget
        self._build_ui()      # Bangun semua komponen UI
        self._connect_async() # Mulai koneksi ke Google di background


    # ── Atur style widget ──────────────────────────────────
    def _setup_style(self):
        style = ttk.Style()
        style.theme_use("clam")
        # Pakai tema "clam" sebagai dasar → bisa dikustomisasi warnanya

        style.configure("Dark.TCombobox",
            fieldbackground=BG_CARD, background=BG_CARD,
            foreground=TEXT_WHITE, selectbackground=ACCENT,
            selectforeground=TEXT_WHITE, arrowcolor=TEXT_WHITE)
        # Atur warna dropdown (Combobox) agar sesuai tema gelap
        # fieldbackground = latar kotak teks
        # foreground = warna teks
        # arrowcolor = warna panah dropdown

        style.configure("Horizontal.TProgressbar",
            background=ACCENT, troughcolor=BG_CARD)
        # Atur warna progress bar (ungu di atas latar gelap)


    # ── Bangun semua UI ────────────────────────────────────
    def _build_ui(self):
        # Header di bagian atas jendela
        hdr = tk.Frame(self.root, bg=BG_PANEL, height=70)
        hdr.pack(fill="x", side="top")
        # fill="x" = isi selebar jendela
        # side="top" = tempel di atas
        hdr.pack_propagate(False)
        # Kunci tinggi 70px, tidak menyesuaikan isi

        tk.Label(hdr, text="🏥 Data Logger Patient Monitor",
                 font=("Segoe UI", 18, "bold"),
                 bg=BG_PANEL, fg=TEXT_WHITE).pack(side="left", padx=20, pady=15)
        # Label judul besar di kiri header

        self.lbl_status = tk.Label(hdr, text="● Connecting...",
                                   font=("Segoe UI", 10),
                                   bg=BG_PANEL, fg=BTN_ORANGE)
        self.lbl_status.pack(side="right", padx=20)
        # Label status koneksi di kanan header
        # Awalnya oranye "Connecting..."
        # Berubah hijau "Connected" kalau berhasil
        # Berubah merah "Failed" kalau gagal

        main = tk.Frame(self.root, bg=BG_DARK)
        main.pack(fill="both", expand=True, padx=10, pady=10)
        # Frame utama yang menampung sidebar + area grafik
        # expand=True = isi sisa ruang yang ada

        self._build_sidebar(main)     # Bangun panel kiri
        self._build_chart_area(main)  # Bangun area grafik kanan


    def _build_sidebar(self, parent):
        sidebar = tk.Frame(parent, bg=BG_PANEL, width=260)
        sidebar.pack(side="left", fill="y", padx=(0, 10))
        # side="left" = tempel di kiri
        # fill="y" = isi tinggi penuh
        sidebar.pack_propagate(False)
        # Kunci lebar 260px

        tk.Label(sidebar, text="📋 Select Recording",
                 font=("Segoe UI", 13, "bold"),
                 bg=BG_PANEL, fg=TEXT_WHITE).pack(pady=(20, 5), padx=15, anchor="w")
        # Judul bagian pilih rekaman, anchor="w" = rata kiri

        tk.Label(sidebar, text="Available recordings:",
                 font=("Segoe UI", 9), bg=BG_PANEL, fg=TEXT_GRAY).pack(padx=15, anchor="w")
        # Keterangan kecil di bawah judul

        self.var_recording = tk.StringVar()
        # Variabel yang menyimpan teks pilihan dropdown saat ini

        self.combo_rec = ttk.Combobox(sidebar, textvariable=self.var_recording,
                                      font=("Segoe UI", 11),
                                      state="readonly", style="Dark.TCombobox", width=24)
        self.combo_rec.pack(padx=15, pady=(8, 5), fill="x")
        # Dropdown pilih rekaman
        # state="readonly" = tidak bisa diketik manual, hanya pilih

        tk.Button(sidebar, text="🔄  Refresh List",
                  font=("Segoe UI", 9), bg=BG_CARD, fg=TEXT_GRAY,
                  relief="flat", cursor="hand2",
                  command=self._refresh_recordings).pack(padx=15, pady=(0, 15), fill="x")
        # Tombol refresh → ambil ulang daftar rekaman dari Sheets
        # cursor="hand2" = kursor berubah jadi tangan saat hover

        self.btn_show = tk.Button(sidebar, text="▶  DISPLAY SIGNALS",
                                  font=("Segoe UI", 11, "bold"),
                                  bg=ACCENT, fg=TEXT_WHITE,
                                  relief="flat", cursor="hand2",
                                  pady=10, command=self._load_and_plot)
        self.btn_show.pack(padx=15, pady=5, fill="x")
        # Tombol utama: load data rekaman dan tampilkan grafik

        tk.Frame(sidebar, bg=BG_CARD, height=2).pack(fill="x", padx=15, pady=15)
        # Garis pemisah (frame tipis setinggi 2px)

        tk.Label(sidebar, text="📊 Recording Info",
                 font=("Segoe UI", 11, "bold"),
                 bg=BG_PANEL, fg=TEXT_WHITE).pack(padx=15, anchor="w")
        # Judul bagian info rekaman

        self.frm_info = tk.Frame(sidebar, bg=BG_PANEL)
        self.frm_info.pack(padx=15, pady=10, fill="x")
        # Frame yang menampung baris-baris info

        self.lbl_rec_id   = self._info_row(self.frm_info, "ID", "-")
        self.lbl_samples  = self._info_row(self.frm_info, "Samples", "-")
        self.lbl_duration = self._info_row(self.frm_info, "Duration", "-")
        self.lbl_start    = self._info_row(self.frm_info, "Start", "-")
        self.lbl_stop     = self._info_row(self.frm_info, "Stop", "-")
        # Buat 5 baris info, awalnya semua "-"
        # Akan diupdate saat rekaman dipilih dan ditampilkan

        tk.Frame(sidebar, bg=BG_CARD, height=2).pack(fill="x", padx=15, pady=15)
        # Garis pemisah

        tk.Label(sidebar, text="💾 Export",
                 font=("Segoe UI", 11, "bold"),
                 bg=BG_PANEL, fg=TEXT_WHITE).pack(padx=15, anchor="w")
        # Judul bagian export

        tk.Button(sidebar, text="🖼  Export PNG",
                  font=("Segoe UI", 10), bg=BTN_BLUE, fg=TEXT_WHITE,
                  relief="flat", cursor="hand2", pady=6,
                  command=lambda: self._export("png")).pack(padx=15, pady=(8, 4), fill="x")
        # Tombol export grafik sebagai gambar PNG

        tk.Button(sidebar, text="📄  Export PDF",
                  font=("Segoe UI", 10), bg=BTN_GREEN, fg=TEXT_WHITE,
                  relief="flat", cursor="hand2", pady=6,
                  command=lambda: self._export("pdf")).pack(padx=15, pady=4, fill="x")
        # Tombol export grafik sebagai PDF

        tk.Button(sidebar, text="📊  Export CSV",
                  font=("Segoe UI", 10), bg=BTN_ORANGE, fg=TEXT_WHITE,
                  relief="flat", cursor="hand2", pady=6,
                  command=self._export_csv).pack(padx=15, pady=4, fill="x")
        # Tombol export data mentah sebagai file CSV

        tk.Frame(sidebar, bg=BG_CARD, height=2).pack(fill="x", padx=15, pady=10)
        # Garis pemisah

        tk.Label(sidebar, text="⚡ Signal Legend",
                 font=("Segoe UI", 10, "bold"),
                 bg=BG_PANEL, fg=TEXT_WHITE).pack(padx=15, pady=(5, 3), anchor="w")
        # Judul legenda warna sinyal

        for sig, col in SIGNAL_COLORS.items():
            row = tk.Frame(sidebar, bg=BG_PANEL)
            row.pack(padx=15, fill="x", pady=1)
            tk.Label(row, text="━", font=("Segoe UI", 12, "bold"),
                     bg=BG_PANEL, fg=col).pack(side="left")
            tk.Label(row, text=sig, font=("Segoe UI", 9),
                     bg=BG_PANEL, fg=TEXT_WHITE).pack(side="left", padx=5)
        # Loop: buat satu baris legenda per sinyal
        # Setiap baris: garis berwarna + nama sinyal


    def _info_row(self, parent, label, value):
        # Helper: buat satu baris info (label kiri + nilai kanan)
        row = tk.Frame(parent, bg=BG_PANEL)
        row.pack(fill="x", pady=2)
        tk.Label(row, text=f"{label}:", font=("Segoe UI", 9),
                 bg=BG_PANEL, fg=TEXT_GRAY, width=8, anchor="w").pack(side="left")
        # Label nama field, misal "ID:" atau "Samples:"
        lbl = tk.Label(row, text=value, font=("Segoe UI", 9, "bold"),
                       bg=BG_PANEL, fg=TEXT_WHITE, anchor="w")
        lbl.pack(side="left")
        # Label nilai, awalnya "-", diupdate nanti
        return lbl
        # Return objek label supaya bisa diupdate dari luar


    def _build_chart_area(self, parent):
        chart_frame = tk.Frame(parent, bg=BG_DARK)
        chart_frame.pack(side="right", fill="both", expand=True)
        # Area grafik di sebelah kanan sidebar, isi sisa ruang

        top = tk.Frame(chart_frame, bg=BG_DARK)
        top.pack(fill="x", pady=(0, 5))
        # Bar di atas grafik: judul + checkbox sinyal

        self.lbl_chart_title = tk.Label(top,
            text="Select a recording and click DISPLAY SIGNALS",
            font=("Segoe UI", 11, "italic"),
            bg=BG_DARK, fg=TEXT_GRAY)
        self.lbl_chart_title.pack(side="left")
        # Judul grafik, berubah saat data di-load atau mouse hover

        self.var_signals = {}
        sig_frame = tk.Frame(top, bg=BG_DARK)
        sig_frame.pack(side="right")
        for sig, col in SIGNAL_COLORS.items():
            var = tk.BooleanVar(value=True)
            # BooleanVar = True/False, default True = semua dicentang
            self.var_signals[sig] = var
            cb = tk.Checkbutton(sig_frame, text=sig, variable=var,
                                font=("Segoe UI", 8), bg=BG_DARK,
                                fg=col, selectcolor=BG_DARK,
                                activebackground=BG_DARK, activeforeground=col,
                                command=self._update_visibility)
            cb.pack(side="left", padx=3)
            # Checkbox per sinyal di kanan atas
            # Kalau dicentang/dilepas → _update_visibility dipanggil
            # → grafik digambar ulang tanpa sinyal yang dilepas

        self.fig = Figure(figsize=(10, 8), facecolor=BG_DARK)
        # Buat objek Figure matplotlib (kanvas kosong untuk grafik)
        # figsize = ukuran dalam inci, facecolor = warna latar

        self.canvas = FigureCanvasTkAgg(self.fig, master=chart_frame)
        # Tempelkan Figure matplotlib ke dalam jendela tkinter
        # Tanpa ini grafik matplotlib tidak bisa tampil di tkinter

        toolbar_frame = tk.Frame(chart_frame, bg=BG_DARK)
        toolbar_frame.pack(fill="x")
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        # Toolbar navigasi bawaan matplotlib:
        # tombol Home, Back, Forward, Pan, Zoom, Save
        self.toolbar.config(bg=BG_DARK)
        self.toolbar.update()

        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        # Tampilkan widget canvas di jendela

        self.canvas.mpl_connect("scroll_event", self._on_scroll)
        # Hubungkan event scroll mouse → fungsi _on_scroll
        # Efek: scroll mouse = zoom in/out grafik

        self.canvas.mpl_connect("motion_notify_event", self._on_mouse_move)
        # Hubungkan event gerak mouse → fungsi _on_mouse_move
        # Efek: nilai sinyal di posisi kursor tampil di judul

        self._show_placeholder()
        # Tampilkan teks panduan sebelum data di-load


    def _show_placeholder(self):
        # Tampilkan teks kosong di tengah grafik sebelum ada data
        self.fig.clear()
        ax = self.fig.add_subplot(111)
        # Tambah satu subplot (1 baris, 1 kolom, posisi 1)
        ax.set_facecolor(BG_DARK)
        ax.text(0.5, 0.5,
                "🏥 Select a recording from the sidebar\n"
                "and click DISPLAY SIGNALS",
                transform=ax.transAxes,
                ha="center", va="center",
                fontsize=16, color=TEXT_GRAY)
        # Tulis teks di tengah grafik
        # transform=ax.transAxes = koordinat 0-1 (0.5,0.5 = tengah)
        ax.set_xticks([]); ax.set_yticks([])
        # Sembunyikan semua angka di sumbu X dan Y
        for spine in ax.spines.values():
            spine.set_visible(False)
        # Sembunyikan garis tepi grafik
        self.canvas.draw()
        # Render ke layar


    # ============================================================
    # KONEKSI KE GOOGLE (dijalankan di background/thread terpisah)
    # ============================================================
    def _connect_async(self):
        def _connect():
            self.spreadsheet = connect_sheets()
            # Jalankan fungsi login di thread terpisah
            # Supaya UI tidak freeze selama proses koneksi
            if self.spreadsheet:
                self.root.after(0, self._on_connected)
                # after(0, fungsi) = jalankan fungsi di thread UI utama
                # Wajib pakai ini saat update UI dari thread lain
            else:
                self.root.after(0, lambda: self.lbl_status.config(
                    text="● Connection failed", fg=BTN_RED))
                # Kalau gagal → ubah label status jadi merah
        threading.Thread(target=_connect, daemon=True).start()
        # Jalankan fungsi _connect di thread baru
        # daemon=True = thread otomatis berhenti kalau app ditutup

    def _on_connected(self):
        self.lbl_status.config(text="● Connected to Sheets", fg=BTN_GREEN)
        # Ubah label status jadi hijau "Connected"
        self._refresh_recordings()
        # Langsung ambil daftar rekaman setelah berhasil konek


    # ============================================================
    # AMBIL DAFTAR REKAMAN DARI SHEET "Recordings"
    # ============================================================
    def _refresh_recordings(self):
        if not self.spreadsheet:
            messagebox.showwarning("Not Connected",
                "Google Sheets connection is not ready.")
            return
        # Kalau belum terhubung → tampilkan peringatan dan berhenti

        def _fetch():
            try:
                ws   = self.spreadsheet.worksheet("Recordings")
                # Buka sheet bernama "Recordings"
                rows = ws.get_all_records()
                # Ambil semua baris sebagai list of dict
                # Contoh: [{"Recording ID":"REC001","Status":"✅ DONE",...}, ...]
                done = [r for r in rows if "DONE" in str(r.get("Status", ""))]
                # Filter: hanya rekaman yang statusnya mengandung kata "DONE"
                # r.get("Status","") = ambil nilai kolom Status, default "" kalau tidak ada
                self.recordings = done
                # Simpan list rekaman yang sudah selesai
                self.root.after(0, self._update_dropdown)
                # Update dropdown di UI thread
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror(
                    "Error", f"Failed to fetch recording list:\n{e}"))
        threading.Thread(target=_fetch, daemon=True).start()
        # Jalankan di background supaya UI tidak freeze

    def _update_dropdown(self):
        if not self.recordings:
            messagebox.showinfo("Info", "No completed recordings found (DONE).")
            return
        options = [
            f"{r['Recording ID']} — {r.get('Total Samples','-')} samples  |  {r.get('Start Time','')[:16]}"
            for r in self.recordings
        ]
        # Buat teks untuk tiap pilihan dropdown
        # Contoh: "REC001 — 1500 samples  |  27/04/2026 10:15"
        # [:16] = ambil 16 karakter pertama dari Start Time (cukup tanggal+jam)
        self.combo_rec["values"] = options
        # Masukkan options ke dropdown
        self.combo_rec.current(0)
        # Pilih item pertama secara otomatis


    # ============================================================
    # LOAD DATA SENSOR DAN TAMPILKAN GRAFIK
    # ============================================================
    def _load_and_plot(self):
        if not self.recordings:
            messagebox.showwarning("No Data", "Recording list is empty.")
            return
        idx = self.combo_rec.current()
        # Ambil index rekaman yang dipilih di dropdown (0, 1, 2, ...)
        if idx < 0:
            messagebox.showwarning("Select Recording",
                "Please select a recording first.")
            return

        rec = self.recordings[idx]
        # Ambil dict info rekaman dari list sesuai index
        self.current_rec = rec
        self.btn_show.config(state="disabled", text="⏳ Loading...")
        # Nonaktifkan tombol sementara supaya tidak diklik dua kali

        def _fetch():
            try:
                sheet_name = rec.get("Sheet Name", f"Data_{rec['Recording ID']}")
                # Ambil nama sheet dari kolom "Sheet Name" di Recordings
                # Misal: "Data_REC001"
                # Kalau kolom kosong, pakai "Data_" + ID sebagai fallback
                ws   = self.spreadsheet.worksheet(sheet_name)
                # Buka sheet data rekaman
                data = ws.get_all_records()
                # Ambil semua baris data sensor
                # Hasilnya: list of dict, misal:
                # [{"Timestamp_ms":1000,"ECG_Lead1":512,...}, ...]
                df = pd.DataFrame(data)
                # Ubah list of dict → DataFrame pandas
                # Seperti mengubah data jadi tabel Excel di Python
                self.current_data = df
                self.root.after(0, lambda: self._plot(df, rec))
                # Kirim ke UI thread untuk digambar
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror(
                    "Error", f"Failed to fetch data:\n{e}"))
                self.root.after(0, self._stop_loading)
        threading.Thread(target=_fetch, daemon=True).start()

    def _stop_loading(self):
        self.btn_show.config(state="normal", text="▶  DISPLAY SIGNALS")
        # Aktifkan kembali tombol, kembalikan teks semula

    def _plot(self, df, rec):
        self._stop_loading()
        # Aktifkan kembali tombol

        # Update label info di sidebar
        self.lbl_rec_id.config(text=rec.get("Recording ID", "-"))
        self.lbl_samples.config(text=str(rec.get("Total Samples", "-")))
        dur = rec.get("Duration (sec)", "-")
        self.lbl_duration.config(text=f"{dur} sec")
        self.lbl_start.config(text=str(rec.get("Start Time", "-"))[:19])
        self.lbl_stop.config(text=str(rec.get("Stop Time",  "-"))[:19])
        # [:19] = potong sampai 19 karakter (cukup untuk "DD/MM/YYYY HH:MM:SS")

        self.lbl_chart_title.config(
            text=f"Recording: {rec.get('Recording ID')}  |  "
                 f"{len(df)} samples  |  {dur} sec",
            fg=TEXT_WHITE)
        # Update judul di atas grafik

        self._draw_signals(df, rec)
        # Gambar grafik sinyal


    # ── Cek apakah sinyal punya data aktif ────────────────
    def _is_active(self, df, col):
        if col not in df.columns:
            return False
            # Kolom tidak ada di DataFrame → tidak aktif
        vals = pd.to_numeric(df[col], errors="coerce").fillna(0)
        # Ubah kolom jadi angka, nilai non-angka jadi NaN, NaN jadi 0
        return float(vals.std()) > ACTIVE_THRESHOLD
        # std() = standar deviasi
        # Kalau > 1.0 → sinyal bervariasi → aktif
        # Kalau <= 1.0 → semua nilai sama/flat → tidak aktif


    # ============================================================
    # GAMBAR GRAFIK SINYAL
    # ============================================================
    def _draw_signals(self, df, rec=None):
        self.fig.clear()
        # Hapus semua grafik lama dari Figure
        self.axes = []
        # Reset list axes

        if rec is None:
            rec = self.current_rec or {}

        # ── Siapkan waktu mulai rekaman ──
        start_str = str(rec.get("Start Time", ""))
        start_dt  = parse_start_time(start_str)
        # Ambil dan parse waktu mulai dari info rekaman
        # Contoh: "27/04/2026 10:15:00" → datetime(2026,4,27,10,15,0)

        # ── Siapkan data waktu (sumbu X) ──
        ts_ms = pd.to_numeric(
            df["Timestamp_ms"] if "Timestamp_ms" in df.columns
            else pd.Series(range(len(df))) * 20,
            errors="coerce"
        ).fillna(0).values
        # Ambil kolom Timestamp_ms dari DataFrame
        # Kalau tidak ada, buat timestamp palsu: 0, 20, 40, 60, ... (tiap 20ms)
        # .values = ubah ke array numpy

        if start_dt:
            abs_times = [start_dt + timedelta(milliseconds=float(t))
                         for t in ts_ms]
            # Konversi timestamp relatif (ms sejak rekaman mulai)
            # → waktu absolut (jam:menit:detik sebenarnya)
            # Contoh: start=10:15:00, ts=5000ms → 10:15:05
            use_abs = True
        else:
            abs_times = ts_ms / 1000.0
            # Fallback: pakai detik relatif kalau start time tidak ada
            use_abs = False

        # ── Tentukan sinyal mana yang ditampilkan ──
        signals_all = list(SIGNAL_COLORS.keys())
        visible = []
        for sig in signals_all:
            col     = SIGNAL_COLS.get(sig, "")
            checked = self.var_signals.get(sig, tk.BooleanVar(value=True)).get()
            # .get() = ambil nilai True/False dari BooleanVar checkbox
            active  = self._is_active(df, col)
            # Cek apakah sinyal punya data aktif (bukan flat)
            if checked and active:
                visible.append(sig)
            # Hanya tampilkan kalau: dicentang DAN punya data

        if not visible:
            # Tidak ada sinyal yang bisa ditampilkan
            ax = self.fig.add_subplot(111)
            ax.set_facecolor(BG_DARK)
            ax.text(0.5, 0.5,
                    "No active signals found.\n"
                    "All signals are flat or unchecked.",
                    transform=ax.transAxes, ha="center", va="center",
                    fontsize=13, color=TEXT_GRAY)
            ax.set_xticks([]); ax.set_yticks([])
            self.canvas.draw()
            return

        n = len(visible)
        # Jumlah sinyal yang akan ditampilkan = jumlah subplot

        # ── Buat subplot untuk setiap sinyal ──
        ax_first = None
        for i, sig in enumerate(visible):
            if i == 0:
                ax = self.fig.add_subplot(n, 1, i + 1)
                # add_subplot(total_baris, total_kolom, posisi)
                # Misal 5 sinyal: add_subplot(5, 1, 1) sampai (5, 1, 5)
                ax_first = ax
                # Simpan axes pertama untuk dijadikan acuan sharex
            else:
                ax = self.fig.add_subplot(n, 1, i + 1, sharex=ax_first)
                # sharex = berbagi sumbu X dengan axes pertama
                # Efek: zoom/pan di satu grafik → SEMUA grafik ikut bergerak

            self.axes.append(ax)

            col   = SIGNAL_COLS[sig]    # Nama kolom di Sheets
            color = SIGNAL_COLORS[sig]  # Warna sinyal ini

            ax.set_facecolor("#1a1a2e")
            # Warna latar tiap subplot (biru sangat gelap)
            ax.tick_params(colors=TEXT_GRAY, labelsize=7)
            # Warna dan ukuran angka di sumbu
            for spine in ax.spines.values():
                spine.set_edgecolor("#2a2a4a")
            # Warna garis tepi subplot

            if col in df.columns:
                df_s = df.copy()
                # Buat salinan DataFrame supaya aslinya tidak berubah

                if "Timestamp_ms" in df.columns:
                    df_s = df_s.sort_values("Timestamp_ms")
                    # Urutkan baris berdasarkan waktu (timestamp terkecil dulu)

                    df_s = df_s.drop_duplicates(subset=["Timestamp_ms"], keep="last")
                    # Hapus baris dengan timestamp sama
                    # keep="last" = simpan yang terakhir, hapus yang sebelumnya
                    # Duplikat terjadi karena beberapa sensor kirim di waktu bersamaan

                    ts_sorted = pd.to_numeric(
                        df_s["Timestamp_ms"], errors="coerce").fillna(0).values
                    # Ambil timestamp yang sudah diurutkan sebagai array angka

                    if start_dt:
                        x_vals = [start_dt + timedelta(milliseconds=float(t))
                                  for t in ts_sorted]
                        # Konversi ke waktu absolut
                    else:
                        x_vals = ts_sorted / 1000.0
                        # Atau pakai detik relatif
                else:
                    x_vals = abs_times

                vals = pd.to_numeric(df_s[col], errors="coerce") \
                         .ffill().fillna(0).values
                # Ambil nilai sinyal dari kolom yang sesuai
                # ffill() = isi nilai kosong/NaN dengan nilai sebelumnya
                # Contoh: [512, NaN, 515] → [512, 512, 515]

                # ── Potong trailing zeros di akhir sinyal ──
                nonzero_idx = np.nonzero(vals)[0]
                # nonzero = cari semua index yang nilainya BUKAN 0
                # [0] = ambil array index-nya
                if len(nonzero_idx) > 0:
                    last_valid = nonzero_idx[-1]
                    # Index terakhir yang nilainya bukan 0
                    vals   = vals  [:last_valid + 1]
                    x_vals = x_vals[:last_valid + 1]
                    # Potong vals dan x_vals sampai index itu
                    # Tujuan: hilangkan garis panjang flat di akhir grafik
                else:
                    ax.set_yticks([])
                    ax.set_ylabel(sig, color=color, fontsize=8, labelpad=4)
                    continue
                    # Semua nilai 0 → skip sinyal ini

                ax.plot(x_vals, vals, color=color, linewidth=0.9, alpha=1.0)
                # Gambar sinyal sebagai garis
                # linewidth=0.9 = garis tipis supaya detail terlihat

                # ── Atur batas Y pakai persentil 2-98% ──
                p_low  = float(np.percentile(vals,  2))
                p_high = float(np.percentile(vals, 98))
                # Persentil 2% = nilai yang 2% data di bawahnya
                # Persentil 98% = nilai yang 98% data di bawahnya
                # Tujuan: abaikan 2% nilai ekstrem di atas dan bawah (noise/spike)
                margin = (p_high - p_low) * 0.25
                if margin < 1:
                    margin = 5
                    # Margin minimum 5 supaya grafik tidak terlalu sempit
                ax.set_ylim(p_low - margin, p_high + margin)
                # Set batas atas dan bawah sumbu Y

            # Grid bergaya ECG (kotak-kotak kecil)
            ax.grid(True, which="minor", color="#1e1e3a", linewidth=0.4, alpha=0.8)
            # Grid minor = garis kecil di antara grid utama
            ax.grid(True, which="major", color="#2e2e5a", linewidth=0.7, alpha=0.9)
            # Grid major = garis utama
            ax.minorticks_on()
            ax.yaxis.set_minor_locator(ticker.AutoMinorLocator(5))
            # Bagi tiap interval utama jadi 5 bagian kecil

            ax.set_ylabel(sig, color=color, fontsize=8, labelpad=4)
            # Label nama sinyal di sumbu Y, warnanya sesuai sinyal

            # ── Format sumbu X ──
            if use_abs:
                if i == n - 1:
                    # Hanya subplot paling bawah yang tampilkan label waktu
                    ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
                    # Format: jam:menit:detik
                    ax.xaxis.set_major_locator(mdates.AutoDateLocator())
                    # Otomatis pilih interval yang cocok
                    plt.setp(ax.xaxis.get_majorticklabels(),
                             rotation=25, ha="right", color=TEXT_GRAY, fontsize=7)
                    # Miringkan label 25 derajat supaya tidak tumpang tindih
                    ax.set_xlabel(
                        f"Waktu  (mulai: {start_str[:19]})",
                        color=TEXT_GRAY, fontsize=8)
                else:
                    plt.setp(ax.xaxis.get_majorticklabels(), visible=False)
                    # Subplot di atas: sembunyikan label X supaya tidak penuh
            else:
                if i == n - 1:
                    ax.set_xlabel("Waktu (detik)", color=TEXT_GRAY, fontsize=8)
                    ax.tick_params(axis="x", colors=TEXT_GRAY, labelsize=7)
                else:
                    plt.setp(ax.xaxis.get_majorticklabels(), visible=False)

        self.fig.patch.set_facecolor(BG_DARK)
        self.fig.subplots_adjust(
            hspace=0.35, left=0.08, right=0.97, top=0.96, bottom=0.10)
        # Atur jarak antar subplot dan margin tepi
        # hspace = jarak vertikal antar subplot

        self._abs_times = abs_times
        self._use_abs   = use_abs
        # Simpan untuk dipakai fungsi hover mouse

        self.canvas.draw()
        # Render semua grafik ke layar


    def _update_visibility(self):
        # Dipanggil saat checkbox sinyal diubah
        if self.current_data is not None and self.current_rec is not None:
            self._draw_signals(self.current_data, self.current_rec)
            # Gambar ulang grafik dengan sinyal yang baru dipilih


    # ── Scroll mouse: zoom sumbu X semua grafik bersamaan ─
    def _on_scroll(self, event):
        if not self.axes or event.inaxes is None:
            return
        # Abaikan kalau tidak ada grafik atau kursor di luar grafik

        ax = self.axes[0]
        # Ambil axes pertama (sharex → semua ikut)

        if self._use_abs:
            xmin_f, xmax_f = ax.get_xlim()
            # Ambil batas X saat ini dalam format float (matplotlib date unit)
            cx = event.xdata
            # Posisi X kursor saat scroll
            if cx is None:
                return
            scale  = 0.82 if event.button == "up" else 1.22
            # Scroll up = zoom in (range mengecil 82%)
            # Scroll down = zoom out (range membesar 122%)
            new_min = cx - (cx - xmin_f) * scale
            new_max = cx + (xmax_f - cx) * scale
            ax.set_xlim(new_min, new_max)
            # Zoom berpusat di posisi kursor
        else:
            xmin, xmax = ax.get_xlim()
            cx = event.xdata
            if cx is None:
                return
            scale = 0.82 if event.button == "up" else 1.22
            ax.set_xlim(cx - (cx - xmin) * scale,
                        cx + (xmax - cx) * scale)

        self.canvas.draw_idle()
        # draw_idle = redraw lebih efisien (tidak block UI)


    # ── Mouse hover: tampilkan nilai sinyal di judul ──────
    def _on_mouse_move(self, event):
        if not self.axes or event.inaxes not in self.axes:
            return
        if self.current_data is None or event.xdata is None:
            return

        df = self.current_data
        if "Timestamp_ms" not in df.columns:
            return

        ts_arr = pd.to_numeric(df["Timestamp_ms"],
                               errors="coerce").fillna(0).values
        # Array semua timestamp dari data

        if self._use_abs:
            import matplotlib.dates as md
            x_dt  = md.num2date(event.xdata).replace(tzinfo=None)
            # Konversi posisi X kursor (float) → datetime
            start_dt = parse_start_time(
                str(self.current_rec.get("Start Time", "")))
            if start_dt is None:
                return
            x_ms = (x_dt - start_dt).total_seconds() * 1000
            # Hitung berapa ms dari awal rekaman ke posisi kursor
        else:
            x_ms = event.xdata * 1000

        idx = int(np.argmin(np.abs(ts_arr - x_ms)))
        # Cari index baris data yang timestampnya paling dekat ke posisi kursor
        # np.abs = nilai absolut selisih
        # argmin = index nilai terkecil = index paling dekat
        row = df.iloc[idx]
        # Ambil baris data tersebut

        rec      = self.current_rec or {}
        start_str = str(rec.get("Start Time", ""))
        start_dt  = parse_start_time(start_str)

        if start_dt:
            abs_t = start_dt + timedelta(milliseconds=float(ts_arr[idx]))
            t_str = abs_t.strftime("%H:%M:%S.") + f"{abs_t.microsecond//1000:03d}"
            # Format waktu: "10:15:03.245" (jam:menit:detik.milidetik)
        else:
            t_str = f"{ts_arr[idx]/1000:.2f}s"

        # Ambil nilai semua sinyal di baris tersebut
        e1 = int(pd.to_numeric(row.get("ECG_Lead1", 0), errors="coerce") or 0)
        e2 = int(pd.to_numeric(row.get("ECG_Lead2", 0), errors="coerce") or 0)
        e3 = int(pd.to_numeric(row.get("ECG_Lead3", 0), errors="coerce") or 0)
        sw = int(pd.to_numeric(row.get("SpO2_Wave", 0), errors="coerce") or 0)
        rw = int(pd.to_numeric(row.get("Resp_Wave", 0), errors="coerce") or 0)

        self.lbl_chart_title.config(
            text=f"🕐 {t_str}  │  ECG1={e1}  ECG2={e2}  ECG3={e3}"
                 f"  │  SpO2={sw}  Resp={rw}",
            fg=ACCENT2)
        # Update judul grafik dengan waktu + nilai sinyal di posisi kursor


    # ── Export grafik sebagai gambar (PNG/PDF) ─────────────
    def _export(self, fmt):
        if self.current_data is None:
            messagebox.showwarning("No Data", "Please display a recording first.")
            return
        rec_id = (self.current_rec.get("Recording ID", "recording")
                  if self.current_rec else "recording")
        fname = filedialog.asksaveasfilename(
            defaultextension=f".{fmt}",
            initialfile=f"{rec_id}_signals.{fmt}",
            filetypes=[(f"{fmt.upper()} files", f"*.{fmt}"),
                       ("All files", "*.*")])
        # Buka dialog "Simpan File" → user pilih lokasi dan nama file
        if fname:
            self.fig.savefig(fname, dpi=200, facecolor=BG_DARK, bbox_inches="tight")
            # Simpan grafik ke file
            # dpi=200 = resolusi tinggi
            # bbox_inches="tight" = tidak ada pinggiran kosong berlebih
            messagebox.showinfo("Saved", f"Chart saved to:\n{fname}")


    # ── Export data mentah sebagai CSV ─────────────────────
    def _export_csv(self):
        if self.current_data is None:
            messagebox.showwarning("No Data", "Please display a recording first.")
            return
        rec_id = (self.current_rec.get("Recording ID", "recording")
                  if self.current_rec else "recording")
        fname = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=f"{rec_id}_data.csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if fname:
            self.current_data.to_csv(fname, index=False)
            # Simpan DataFrame pandas → file CSV
            # index=False = tidak ikutkan nomor baris otomatis pandas
            messagebox.showinfo("Saved", f"CSV data saved to:\n{fname}")


# ============================================================
# TITIK MASUK PROGRAM
# Kode di bawah dijalankan saat file ini dieksekusi langsung
# ============================================================
if __name__ == "__main__":
    root = tk.Tk()
    # Buat jendela utama tkinter

    app = PatientMonitorApp(root)
    # Buat objek aplikasi → otomatis bangun UI dan mulai koneksi ke Google

    root.mainloop()
    # Jalankan loop event tkinter
    # Program terus berjalan di sini sampai jendela ditutup
    # Loop ini yang menangani: klik tombol, scroll, ketik, dll