# main.py
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from typing import List
from pydantic import BaseModel
from db import connect_pool, close_pool, fetch_all, execute

app = FastAPI(title="MyRemote API", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], allow_credentials=True,
    allow_methods=["*"], allow_headers=["*"],
)

@app.on_event("startup")
async def _startup():
    await connect_pool()

@app.on_event("shutdown")
async def _shutdown():
    await close_pool()

class Signal(BaseModel):
    id: int
    signal_name: str
    ir_signal: str
    uploader: str

@app.get("/signals", response_model=List[Signal])
async def signals():
    rows = await fetch_all(
        "SELECT id, signal_name, ir_signal, uploader FROM remote ORDER BY id ASC"
    )
    return rows

class RenameIn(BaseModel):
    signal_name: str

@app.put("/signals/{signal_id}/name", response_model=Signal)
async def rename_signal(signal_id: int, body: RenameIn):
    # 이름 변경 (MySQL placeholder: %s)
    affected = await execute(
        "UPDATE remote SET signal_name = %s WHERE id = %s",
        body.signal_name, signal_id
    )
    # 변경 후 조회
    rows = await fetch_all(
        "SELECT id, signal_name, ir_signal, uploader FROM remote WHERE id = %s",
        signal_id
    )
    if not rows:
        raise HTTPException(status_code=404, detail="Signal not found")
    return rows[0]
