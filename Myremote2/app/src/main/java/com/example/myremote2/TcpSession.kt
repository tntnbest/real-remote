package com.example.myremote2

import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow
import java.io.BufferedReader
import java.io.BufferedWriter
import java.io.InputStreamReader
import java.io.OutputStreamWriter
import java.net.InetSocketAddress
import java.net.Socket
import java.nio.charset.Charset

object TcpSession {
    // 🔧 네 환경 값으로 조정
    private const val HOST = "10.10.14.84"
    private const val PORT = 5000
    private const val CONNECT_TIMEOUT_MS = 4000
    private const val SO_TIMEOUT_MS = 0          // 0 = read 무기한 대기
    private const val TERM = "\n"                // 서버가 CRLF면 "\r\n"
    private val CS: Charset = Charsets.UTF_8

    private const val LOGIN_ID = "LDH_SMP"
    private const val LOGIN_PW = "PASSWD"

    @Volatile private var socket: Socket? = null
    @Volatile private var writer: BufferedWriter? = null
    @Volatile private var readerJob: Job? = null

    @Volatile var isLoggedIn: Boolean = false
        private set

    private val _incoming = MutableSharedFlow<String>(extraBufferCapacity = 64)
    val incoming: SharedFlow<String> = _incoming

    suspend fun connectAndLogin(): Result<Unit> = withContext(Dispatchers.IO) {
        runCatching {
            if (socket?.isConnected == true && writer != null && isLoggedIn) return@runCatching

            val s = Socket().apply {
                soTimeout = SO_TIMEOUT_MS
                tcpNoDelay = true
                keepAlive = true
                connect(InetSocketAddress(HOST, PORT), CONNECT_TIMEOUT_MS)
            }

            val w = BufferedWriter(OutputStreamWriter(s.getOutputStream(), CS))
            val r = BufferedReader(InputStreamReader(s.getInputStream(), CS))

            // 로그인 전송
            w.write("[$LOGIN_ID]:$LOGIN_PW")
            w.write(TERM)
            w.flush()

            socket = s
            writer = w
            isLoggedIn = true

            // 수신 루프 시작 (라인 단위)
            readerJob?.cancel()
            readerJob = CoroutineScope(Dispatchers.IO).launch {
                try {
                    while (isActive) {
                        val line = r.readLine() ?: break
                        _incoming.tryEmit(line)
                    }
                } catch (_: Throwable) {
                    // 소켓 종료 등
                } finally {
                    isLoggedIn = false
                }
            }
        }
    }

    suspend fun send(message: String): Result<Unit> = withContext(Dispatchers.IO) {
        runCatching {
            val w = writer ?: error("Not connected")
            w.write(message)
            w.write(TERM)
            w.flush()
        }
    }

    suspend fun close(): Result<Unit> = withContext(Dispatchers.IO) {
        runCatching {
            readerJob?.cancel()
            readerJob = null
            isLoggedIn = false
            writer?.run {
                flush()
                close()
            }
            socket?.close()
            writer = null
            socket = null
        }
    }
}
