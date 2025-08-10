from fastapi import FastAPI, Request, UploadFile, File, Form
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
import shutil
import os
import sqlite3
import subprocess
import base64
import rclpy
from sensor_msgs.msg import CompressedImage
import redis
import json

app = FastAPI()
templates = Jinja2Templates(directory="templates")

VIDEO_DIR = "video_storage"
DB_PATH = "videos.db"
os.makedirs(VIDEO_DIR, exist_ok=True)

# DB初期化
with sqlite3.connect(DB_PATH) as conn:
    cur = conn.cursor()
    cur.execute("""CREATE TABLE IF NOT EXISTS videos (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        filename TEXT NOT NULL,
        filepath TEXT NOT NULL,
        uploaded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    )""")
    conn.commit()

def fetch_videos():
    with sqlite3.connect(DB_PATH) as conn:
        cur = conn.cursor()
        cur.execute("SELECT id, filename, uploaded_at FROM videos ORDER BY uploaded_at DESC")
        return cur.fetchall()

def insert_video(filename, filepath):
    with sqlite3.connect(DB_PATH) as conn:
        cur = conn.cursor()
        cur.execute("INSERT INTO videos (filename, filepath) VALUES (?, ?)", (filename, filepath))
        conn.commit()
        return cur.lastrowid

@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    videos = fetch_videos()
    return templates.TemplateResponse("index.html", {"request": request, "videos": videos})

@app.post("/upload", response_class=HTMLResponse)
async def upload(request: Request, file: UploadFile = File(...)):
    # DBに仮登録してIDを取得
    temp_name = "temp"
    video_id = insert_video(temp_name, "")
    video_filename = f"{video_id}.mp4"
    thumbnail_filename = f"{video_id}.jpg"
    video_path = os.path.join(VIDEO_DIR, video_filename)
    thumbnail_path = os.path.join(VIDEO_DIR, thumbnail_filename)
    temp_path = os.path.join(VIDEO_DIR, f"temp_upload_{video_id}")

    # 一時ファイルとして保存
    with open(temp_path, "wb") as buffer:
        shutil.copyfileobj(file.file, buffer)

    # ffmpegで動画を320x160のmp4に変換
    subprocess.run(
        [
            "ffmpeg", "-y", "-i", temp_path,
            "-vf", "scale=320:160",
            "-r", "10",  # 10Hz（10fps）に変換
            "-c:v", "libx264", "-preset", "veryfast", "-crf", "28",
            "-an", video_path
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    # ffmpegでサムネイル生成
    subprocess.run(
        ["ffmpeg", "-y", "-i", video_path, "-ss", "00:00:01.000", "-vframes", "1", "-vf", "scale=320:160", thumbnail_path],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    # DBのファイル名・パスを更新
    with sqlite3.connect(DB_PATH) as conn:
        cur = conn.cursor()
        cur.execute("UPDATE videos SET filename=?, filepath=? WHERE id=?", (video_filename, video_path, video_id))
        conn.commit()

    # 一時ファイル削除
    os.remove(temp_path)

    # アップロード後にトップページへリダイレクト
    return RedirectResponse(url="/", status_code=303)

@app.post("/delete/{video_id}")
async def delete_video(video_id: int):
    # DBからファイル情報取得
    with sqlite3.connect(DB_PATH) as conn:
        cur = conn.cursor()
        cur.execute("SELECT filename, filepath FROM videos WHERE id=?", (video_id,))
        row = cur.fetchone()
        if row:
            filename, filepath = row
            # 動画ファイル削除
            if os.path.exists(filepath):
                os.remove(filepath)
            # サムネイル削除
            thumb_path = os.path.join(VIDEO_DIR, f"{video_id}.jpg")
            if os.path.exists(thumb_path):
                os.remove(thumb_path)
            # DBから削除
            cur.execute("DELETE FROM videos WHERE id=?", (video_id,))
            conn.commit()
    return RedirectResponse(url="/", status_code=303)

# Redisクライアント
r = redis.Redis(host='localhost', port=6379, db=0)

@app.post("/frame")
async def receive_frame(request: Request):
    data = await request.json()
    image_b64 = data["image"].split(",")[1]
    img_bytes = base64.b64decode(image_b64)
    # Redisに画像データを保存（バイナリはbase64で）
    r.set("video_frame", data["image"])
    return {"status": "ok"}

@app.post("/ui_command")
async def ui_command(request: Request):
    cmd = await request.json()
    r.set("ui_command", json.dumps(cmd))
    return {"status": "ok"}

@app.get("/imu")
async def get_imu():
    imu_json = r.get("imu_quat")
    if imu_json:
        return json.loads(imu_json)
    return {}

app.mount("/static", StaticFiles(directory=VIDEO_DIR), name="static")

# micro-ROS2ノードのセットアップ（別スレッドで起動推奨）
rclpy.init()
node = rclpy.create_node('web_frame_bridge')
pub = node.create_publisher(CompressedImage, 'video_frames', 10)
