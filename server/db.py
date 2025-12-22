# db.py
import os
import aiomysql
from dotenv import load_dotenv
load_dotenv()

DB_HOST = os.getenv("DB_HOST", "127.0.0.1")
DB_PORT = int(os.getenv("DB_PORT", "3306"))
DB_USER = os.getenv("DB_USER", "root")
DB_PASSWORD = os.getenv("DB_PASSWORD", "ehdgus77")
DB_NAME = os.getenv("DB_NAME", "kdk")

pool: aiomysql.Pool | None = None

async def connect_pool():
    global pool
    if pool is None:
        pool = await aiomysql.create_pool(
            host=DB_HOST, port=DB_PORT, user=DB_USER, password=DB_PASSWORD,
            db=DB_NAME, autocommit=True, minsize=1, maxsize=10, charset="utf8mb4"
        )

async def close_pool():
    global pool
    if pool:
        pool.close()
        await pool.wait_closed()
        pool = None

async def fetch_all(q: str, *params):
    assert pool is not None, "DB pool not initialized"
    async with pool.acquire() as conn:
        async with conn.cursor(aiomysql.DictCursor) as cur:
            await cur.execute(q, params)
            return await cur.fetchall()

async def execute(q: str, *params) -> int:
    assert pool is not None, "DB pool not initialized"
    async with pool.acquire() as conn:
        async with conn.cursor() as cur:
            await cur.execute(q, params)
            return cur.rowcount
