import tkinter as tk
from tkinter import Label, Frame, Button, StringVar, OptionMenu
from ultralytics import YOLO
import cv2
from PIL import Image, ImageTk
import paho.mqtt.client as mqtt
import threading, json, time

# ==== batas berat (samakan dgn firmware) ====
MIN_WEIGHT = 5.0
MED_MIN    = 100.0
BIG_MIN    = 150.0

def has_cuda():
    try:
        import torch
        return torch.cuda.is_available()
    except Exception:
        return False

def weight_band(gram: float):
    if gram is None: return None
    if gram < MIN_WEIGHT: return None
    if gram >= BIG_MIN:   return "besar"
    if gram >= MED_MIN:   return "sedang"
    return "kecil"

class YoloApp:
    def __init__(self, model_path, cam_index=0, mqtt_broker="localhost"):
        # ==== Model ====
        self.model = YOLO(model_path)
        self.device = 0 if has_cuda() else "cpu"
        try:
            if self.device == 0:
                self.model.to("cuda")
        except Exception:
            self.device = "cpu"

        # ==== Kamera ====
        self.backend = cv2.CAP_DSHOW  # Windows
        self.cam_index = cam_index
        self.cap = None
        self.reopen_camera(self.cam_index)

        # ==== MQTT ====
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        try:
            self.client.connect(mqtt_broker, 1883, 60)
        except Exception as e:
            print(f"[WARN] MQTT connect gagal: {e}")

        # ==== GUI ====
        self.root = tk.Tk()
        self.root.title("Sistem Penyortir Tomat (MQTT)")
        main_frame = Frame(self.root); main_frame.pack()

        # Video
        self.label = Label(main_frame); self.label.grid(row=0, column=0)

        # Panel Status
        status_frame = Frame(main_frame, relief="groove", borderwidth=2)
        status_frame.grid(row=0, column=1, padx=10, pady=10, sticky="n")

        Label(status_frame, text="Load Cell (gram):", font=("Arial", 12, "bold")).pack()
        self.loadcell_label = Label(status_frame, text="0.00", font=("Arial", 12), fg="blue"); self.loadcell_label.pack()

        Label(status_frame, text="Kategori:", font=("Arial", 12, "bold")).pack()
        self.kategori_label = Label(status_frame, text="unknown", font=("Arial", 12), fg="purple"); self.kategori_label.pack()

        Label(status_frame, text="Kategori Berat:", font=("Arial", 12, "bold")).pack()
        self.kat_berat_label = Label(status_frame, text="-", font=("Arial", 12), fg="darkgreen"); self.kat_berat_label.pack()

        # NEW: indikator benda tak dikenal (IR1)
        Label(status_frame, text="Benda tak dikenal:", font=("Arial", 12, "bold")).pack()
        self.unknown_label = Label(status_frame, text="Tidak", font=("Arial", 12), bg="red", fg="white")
        self.unknown_label.pack()

        Label(status_frame, text="State:", font=("Arial", 12, "bold")).pack()
        self.state_label = Label(status_frame, text="-", font=("Arial", 11)); self.state_label.pack()

        Label(status_frame, text="Target Servo:", font=("Arial", 12, "bold")).pack()
        self.target_label = Label(status_frame, text="0", font=("Arial", 11)); self.target_label.pack()

        Label(status_frame, text="Infrared:", font=("Arial", 12, "bold")).pack()
        self.ir_labels = []
        for i in range(5):
            lbl = Label(status_frame, text=f"IR {i+1}", width=12, bg="red", fg="white"); lbl.pack()
            self.ir_labels.append(lbl)

        Label(status_frame, text="Servo:", font=("Arial", 12, "bold")).pack()
        self.servo_labels = []
        for i in range(5):
            lbl = Label(status_frame, text=f"Servo {i+1}", width=12, bg="red", fg="white"); lbl.pack()
            self.servo_labels.append(lbl)

        # === FPS LABEL (baru) ===
        Label(status_frame, text="FPS (video / infer):", font=("Arial", 12, "bold")).pack(pady=(8,0))
        self.fps_label = Label(status_frame, text="0.0 / 0.0", font=("Arial", 12), fg="brown")
        self.fps_label.pack()

        # Panel pilih kamera
        cam_frame = Frame(main_frame, relief="groove", borderwidth=2)
        cam_frame.grid(row=1, column=0, padx=10, pady=10, sticky="w")
        Label(cam_frame, text="Camera Index:", font=("Arial", 11, "bold")).pack(side="left", padx=4)
        self.cam_var = StringVar(value=str(self.cam_index))
        self.available_cams = self.enumerate_cameras(range(0, 6)) or ["0","1","2","3"]
        self.cam_menu = OptionMenu(cam_frame, self.cam_var, *self.available_cams); self.cam_menu.pack(side="left", padx=4)
        Button(cam_frame, text="Switch", command=lambda: self.reopen_camera(self.cam_var.get())).pack(side="left", padx=4)
        self.root.bind("[", self.prev_cam); self.root.bind("]", self.next_cam)

        Button(self.root, text="Quit", command=self.on_close, bg="red", fg="white").pack()

        # ==== Performansi ====
        self.imgsz = 512
        self.conf_thres = 0.40
        self.frame_skip = 2
        self._frame_counter = 0
        self._last_annotated = None

        # ==== Status dari ESP32 ====
        self.last_state = "-"           # "IDLE", "ARMED", ...
        self.last_servo = [0,0,0,0,0]
        self.last_weight = 0.0
        self.last_band = None
        self.servo1_active_threshold = 45

        # ==== Latching / Siklus ====
        self.camera_warmup_until = time.time() + 1.0

        self.cooldown_after_servo1_s = 4.0
        self.cooldown_until = 0.0

        self.post_idle_delay_s = 2.0
        self.post_idle_until = 0.0

        self.armed = False
        self.latched = False
        self.latched_kategori = None
        self.latched_band = None

        # Publisher (YOLO)
        self.last_pub_time = 0.0
        self.pub_interval = 0.20

        # >>>>>> ATURAN KHUSUS SERVO 2-5 <<<<<<
        self.servo_active_threshold = 45        # ambang aktif untuk servo 2-5
        self.return_debounce_s = 1.0            # cegah spam return
        self._servo_return_cooldown = [0.0]*6   # pakai index 1..5

        # ==== FPS state (baru) ====
        self.video_fps_ema = 0.0
        self.infer_fps_ema = 0.0
        self.ema_alpha = 0.12     # smoothing; makin besar makin responsif
        self._last_frame_time = time.time()

        # Threads
        threading.Thread(target=self.camera_loop, daemon=True).start()
        threading.Thread(target=self.mqtt_loop, daemon=True).start()

    # ===== Kamera utils =====
    def enumerate_cameras(self, indices):
        found = []
        for i in indices:
            cap = cv2.VideoCapture(int(i), self.backend)
            ok = cap.isOpened()
            if ok: ok, _ = cap.read()
            cap.release()
            if ok: found.append(str(i))
        print(f"[INFO] Kamera terdeteksi: {found}")
        return found

    def reopen_camera(self, new_index):
        try:
            if self.cap is not None: self.cap.release()
        except: pass
        self.cam_index = int(new_index)
        self.cap = cv2.VideoCapture(self.cam_index, self.backend)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        print(f"[INFO] Kamera aktif di index {self.cam_index}")

    def prev_cam(self, _evt=None):
        if not self.available_cams: return
        cur = self.cam_var.get()
        try: idx = self.available_cams.index(cur)
        except ValueError: idx = 0
        idx = max(0, idx-1)
        self.cam_var.set(self.available_cams[idx])
        self.reopen_camera(self.cam_var.get())

    def next_cam(self, _evt=None):
        if not self.available_cams: return
        cur = self.cam_var.get()
        try: idx = self.available_cams.index(cur)
        except ValueError: idx = 0
        idx = min(len(self.available_cams)-1, idx+1)
        self.cam_var.set(self.available_cams[idx])
        self.reopen_camera(self.cam_var.get())

    # ====== Siklus/gating ======
    def _is_idle(self): return self.last_state == "IDLE"
    def _servo1_active(self):
        try: return int(self.last_servo[0]) >= self.servo1_active_threshold
        except: return False

    def _maybe_arm(self):
        now = time.time()
        self.armed = ( self._is_idle()
                       and not self._servo1_active()
                       and now >= self.cooldown_until
                       and now >= self.post_idle_until
                       and not self.latched )

    def _reset_cycle(self, full_ui=True):
        self.latched = False
        self.latched_kategori = None
        self.latched_band = None
        if full_ui:
            self.kategori_label.config(text="unknown")
            self.kat_berat_label.config(text="-")

    def _trigger_cooldown_after_servo1(self):
        self.cooldown_until = time.time() + self.cooldown_after_servo1_s
        self._reset_cycle(full_ui=True)

    def _on_back_to_idle(self):
        self.post_idle_until = time.time() + self.post_idle_delay_s

    def _ui_show_latched_or_blank(self):
        if self.latched:
            self.kategori_label.config(text=self.latched_kategori or "unknown")
            self.kat_berat_label.config(text=self.latched_band or "-")
        else:
            self.kategori_label.config(text="unknown")
            self.kat_berat_label.config(text="-")

    # ===== MQTT =====
    def on_connect(self, client, userdata, flags, rc):
        try:
            client.subscribe("tomat/status")
            print("[INFO] MQTT connected & subscribed to tomat/status")
        except Exception as e:
            print("Subscribe error:", e)

    # ===== Kamera + YOLO =====
    def camera_loop(self):
        miss = 0; t0 = time.time(); frames = 0
        while True:
            if self.cap is None:
                time.sleep(0.03); continue

            if not self.cap.grab():
                miss += 1
                if miss % 50 == 0: print(f"[WARN] Grab gagal (idx={self.cam_index}, miss={miss})")
                time.sleep(0.01); continue

            ret, frame = self.cap.retrieve()
            if not ret or frame is None:
                miss += 1
                if miss % 50 == 0: print(f"[WARN] Retrieve gagal (idx={self.cam_index}, miss={miss})")
                time.sleep(0.01); continue
            miss = 0; frames += 1

            # === Hitung video FPS (per frame) ===
            now_frame = time.time()
            dt_frame = now_frame - self._last_frame_time
            self._last_frame_time = now_frame
            if dt_frame > 0:
                inst_video_fps = 1.0 / dt_frame
                self.video_fps_ema = inst_video_fps if self.video_fps_ema == 0 else \
                    (1 - self.ema_alpha) * self.video_fps_ema + self.ema_alpha * inst_video_fps

            run_infer = (self._frame_counter % self.frame_skip == 0); self._frame_counter += 1
            annotated = None; kategori = "unknown"

            if run_infer:
                # === Waktu inferensi untuk FPS infer ===
                t_inf0 = time.time()
                try:
                    results = self.model.predict(source=frame, imgsz=self.imgsz,
                                                 conf=self.conf_thres, verbose=False,
                                                 device=self.device)
                except Exception as e:
                    print(f"[ERR] YOLO inference error: {e}")
                    results = None
                t_inf1 = time.time()
                dt_inf = t_inf1 - t_inf0
                if dt_inf > 0:
                    inst_infer_fps = 1.0 / dt_inf
                    self.infer_fps_ema = inst_infer_fps if self.infer_fps_ema == 0 else \
                        (1 - self.ema_alpha) * self.infer_fps_ema + self.ema_alpha * inst_infer_fps

                if results and len(results) > 0:
                    r0 = results[0]
                    try: annotated = r0.plot(); self._last_annotated = annotated
                    except: annotated = frame; self._last_annotated = annotated

                    try:
                        if hasattr(r0,"boxes") and r0.boxes is not None and len(r0.boxes)>0:
                            confs = r0.boxes.conf.tolist()
                            idx = int(max(range(len(confs)), key=lambda i: confs[i]))
                            cls_id = int(r0.boxes.cls[idx])
                            name = r0.names.get(cls_id, None) if hasattr(r0,"names") else None
                            kategori = "baik" if (name is not None and str(name).lower()=="baik") else "reject"
                    except Exception as e:
                        print(f"[WARN] Parse boxes: {e}")
                else:
                    annotated = self._last_annotated if self._last_annotated is not None else frame
            else:
                annotated = self._last_annotated if self._last_annotated is not None else frame

            # === PUBLISH (gabung dgn berat) ===
            now = time.time()
            self._maybe_arm()

            ready_common = ( now >= self.camera_warmup_until and
                             self.armed and
                             now >= self.cooldown_until and
                             now >= self.post_idle_until )

            band_ok = (self.last_band is not None)

            if (kategori != "unknown" and ready_common and band_ok and not self.latched):
                try:
                    self.client.publish("tomat/kategori", kategori)   # baik → 2/3/4; reject → 5 (di firmware)
                    self.latched = True
                    self.latched_kategori = kategori
                    self.latched_band = self.last_band
                    self.last_pub_time = now
                    self.armed = False
                    self._ui_show_latched_or_blank()
                    print(f"[PUB] kategori={kategori} band={self.latched_band}  (LATCHED)")
                except Exception as e:
                    print("Publish error:", e)

            # === Overlay FPS di video (baru) ===
            try:
                overlay_text = f"FPS {self.video_fps_ema:.1f} | Infer {self.infer_fps_ema:.1f}"
                cv2.putText(annotated, overlay_text, (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 255, 50), 2, cv2.LINE_AA)
            except Exception as e:
                print(f"[WARN] Overlay FPS: {e}")

            # Tampilkan video + update label FPS
            try:
                rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
                imgtk = ImageTk.PhotoImage(image=Image.fromarray(rgb))
                self.root.after(0, lambda imgtk=imgtk: self.label.config(image=imgtk))
                self.label.imgtk = imgtk

                self.root.after(0, lambda:
                    self.fps_label.config(text=f"{self.video_fps_ema:.1f} / {self.infer_fps_ema:.1f}"))
            except Exception as e:
                print(f"[WARN] Render: {e}")

            # (opsional) log per 60 frame
            if frames % 60 == 0:
                t1 = time.time()
                fps = 60.0 / (t1 - t0) if (t1 - t0) > 0 else 0.0
                print(f"[INFO] FPS ~ {fps:.1f} (device={self.device}, imgsz={self.imgsz}, skip={self.frame_skip})")
                t0 = t1

            time.sleep(0.001)

    def mqtt_loop(self):
        try:
            self.client.loop_forever()
        except Exception as e:
            print(f"[WARN] MQTT loop error: {e}")

    # ===== Helper: perintah return untuk servo 2-5 =====
    def _publish_servo_return(self, servo_no: int):
        try:
            payload = {"servo": int(servo_no), "action": "return"}
            self.client.publish("tomat/servo/return", json.dumps(payload))
            print(f"[CMD] Return servo {servo_no} (IR{servo_no}=HIGH)")
        except Exception as e:
            print("Publish return error:", e)

    def _maybe_return_servo_by_ir(self, ir_vals):
        """
        Aturan khusus: servo 2-5 kembali jika IR pasangannya HIGH.
        Mapping: servo2<->IR2, servo3<->IR3, servo4<->IR4, servo5<->IR5.
        """
        now = time.time()
        for idx in range(1, 5):  # 1..4 => servo 2..5
            try:
                servo_deg = int(self.last_servo[idx])
                ir_high = int(ir_vals[idx]) == 1
            except Exception:
                continue
            active = servo_deg >= self.servo_active_threshold
            if active and ir_high and now >= self._servo_return_cooldown[idx]:
                self._servo_return_cooldown[idx] = now + self.return_debounce_s
                self._publish_servo_return(idx + 1)

    # ===== Status dari ESP32 =====
    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())

            # Berat & kategori berat (lokal)
            berat = float(data.get("Berat", 0))
            self.last_weight = berat
            band_local = weight_band(berat)
            self.last_band = band_local
            self.loadcell_label.config(text=f"{berat:.2f}")

            # State & target
            prev_state = self.last_state
            self.last_state = data.get("State", "-")
            self.state_label.config(text=self.last_state)
            self.target_label.config(text=str(data.get("TargetServo", 0)))

            # IR
            ir = data.get("IR", [])
            for i, val in enumerate(ir):
                if i < len(self.ir_labels):
                    self.ir_labels[i].config(bg=("green" if val == 1 else "red"))

            # Servo & pantau Servo1
            servo = data.get("Servo", [])
            if isinstance(servo, list):
                self.last_servo = servo + [0]*(5-len(servo))
            for i, deg in enumerate(self.last_servo[:5]):
                try:
                    self.servo_labels[i].config(bg=("green" if int(deg) >= 45 else "red"))
                except:
                    self.servo_labels[i].config(bg="red")

            # Return otomatis servo 2..5 saat IR pasangan terpicu
            self._maybe_return_servo_by_ir(ir)

            # Tampilkan latched (jika ada)
            self._ui_show_latched_or_blank()

            # --- Benda tak dikenal dari IR1 / Unknown flag ---
            try:
                unknown_flag = int(data.get("Unknown", -1))
                if unknown_flag not in (0, 1):
                    raise ValueError
            except Exception:
                # fallback ke IR[0] bila firmware lama
                try:
                    unknown_flag = 1 if int(ir[0]) == 1 else 0
                except Exception:
                    unknown_flag = 0

            if unknown_flag == 1:
                self.unknown_label.config(text="Ya", bg="green")
                # Jika belum latched oleh YOLO, tampilkan 'benda tak dikenal'
                if not self.latched:
                    self.kategori_label.config(text="benda tak dikenal")
            else:
                self.unknown_label.config(text="Tidak", bg="red")
                if not self.latched:
                    self.kategori_label.config(text="unknown")

            # Jika Servo1 aktif → reset & cooldown (aturan existing)
            if self._servo1_active():
                self._trigger_cooldown_after_servo1()
                print("[INFO] Servo1 aktif → RESET & cooldown")

            # Saat kembali ke IDLE dari state lain → post-IDLE delay
            if self.last_state == "IDLE" and prev_state != "IDLE":
                self._on_back_to_idle()
                print("[INFO] Kembali IDLE → post-IDLE delay 2s")

        except Exception as e:
            print("Parse error:", e)

    # ===== Close =====
    def on_close(self):
        try: self.client.disconnect()
        except: pass
        try:
            if self.cap is not None: self.cap.release()
        except: pass
        self.root.quit(); self.root.destroy()

    def run(self): self.root.mainloop()

if __name__=="__main__":
    app = YoloApp(
        model_path="D:/tugasakhirTA/fold4/weights/best.pt",
        cam_index=1,
        mqtt_broker="10.31.203.73"
    )
    app.run()
