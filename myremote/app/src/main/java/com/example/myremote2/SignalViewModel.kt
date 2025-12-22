package com.example.myremote2

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.flow.collectLatest
import java.net.URLEncoder
import java.nio.charset.StandardCharsets

sealed interface SignalsUiState {
    object Loading : SignalsUiState
    data class Success(val items: List<Signal>) : SignalsUiState
    data class Error(val message: String) : SignalsUiState
}

sealed interface ConnState {
    object Idle : ConnState
    object Connecting : ConnState
    object Connected : ConnState
    data class Failed(val message: String) : ConnState
}

data class FavItem(val id: Int, val name: String)
enum class SaveMode { WIFI, BLE }

class SignalViewModel(app: Application) : AndroidViewModel(app) {

    private val applicationCtx = app.applicationContext

    private val _uiState = MutableStateFlow<SignalsUiState>(SignalsUiState.Loading)
    val uiState: StateFlow<SignalsUiState> = _uiState

    private val _connState = MutableStateFlow<ConnState>(ConnState.Idle)
    val connState: StateFlow<ConnState> = _connState

    // 즐겨찾기: 24칸(4x6) 유지
    private val capacity = 24
    private val _fav1 = MutableStateFlow(List<FavItem?>(capacity) { null })
    private val _fav2 = MutableStateFlow(List<FavItem?>(capacity) { null })
    val fav1: StateFlow<List<FavItem?>> = _fav1
    val fav2: StateFlow<List<FavItem?>> = _fav2

    private val _renameDialogTarget = MutableStateFlow<Int?>(null)
    val renameDialogTarget: StateFlow<Int?> = _renameDialogTarget

    init {
        refresh()

        // 저장된 즐겨찾기 복구(자동 반영)
        viewModelScope.launch {
            FavoritesStore.flow(applicationCtx, which = 1, capacity = capacity).collectLatest {
                _fav1.value = it
            }
        }
        viewModelScope.launch {
            FavoritesStore.flow(applicationCtx, which = 2, capacity = capacity).collectLatest {
                _fav2.value = it
            }
        }

        viewModelScope.launch {
            TcpSession.incoming.collect { line ->
                val msg = line.trim()
                if (msg.startsWith("[LDH_SQL]IRSAVE")) {
                    val id = msg.substringAfter("@", "").toIntOrNull()
                    if (id != null) _renameDialogTarget.value = id
                }
            }
        }
    }

    fun refresh() {
        _uiState.value = SignalsUiState.Loading
        viewModelScope.launch {
            runCatching { ApiClient.api.getSignals() }
                .onSuccess { _uiState.value = SignalsUiState.Success(it) }
                .onFailure { e -> _uiState.value = SignalsUiState.Error(e.message ?: "Failed to load") }
        }
    }

    fun connect() {
        if (_connState.value is ConnState.Connected || _connState.value is ConnState.Connecting) return
        _connState.value = ConnState.Connecting
        viewModelScope.launch {
            TcpSession.connectAndLogin()
                .onSuccess { _connState.value = ConnState.Connected }
                .onFailure { e -> _connState.value = ConnState.Failed(e.message ?: "Connect failed") }
        }
    }

    fun close() {
        viewModelScope.launch {
            TcpSession.close()
            _connState.value = ConnState.Idle
        }
    }

    /** 신호 전송 */
    fun sendGetIr(id: Int, onDone: (Boolean, String) -> Unit = { _, _ -> }) {
        viewModelScope.launch {
            if (_connState.value !is ConnState.Connected) connect()
            val cmd = "[LDH_SQL]GETIR@$id"
            val res = TcpSession.send(cmd)
            if (res.isSuccess) onDone(true, "TX OK: $cmd")
            else {
                val msg = res.exceptionOrNull()?.message ?: "TX failed"
                _connState.value = ConnState.Failed(msg)
                onDone(false, msg)
            }
        }
    }

    /** 즐겨찾기 추가: which=1|2, 비어있는 첫 슬롯에 채움 + 저장 */
    fun addFavorite(which: Int, signal: Signal): Pair<Boolean, String> {
        val flow = if (which == 1) _fav1 else _fav2
        val list = flow.value.toMutableList()

        if (list.any { it?.id == signal.id }) return false to "이미 즐겨찾기에 있음"

        val idx = list.indexOfFirst { it == null }
        if (idx == -1) return false to "빈 슬롯이 없음"

        list[idx] = FavItem(signal.id, signal.signal_name)
        flow.value = list
        persist(which, list)
        return true to "추가됨 (슬롯 ${idx + 1})"
    }

    /** 슬롯 전송 */
    fun sendFavorite(which: Int, index: Int, onDone: (Boolean, String) -> Unit) {
        val item = (if (which == 1) _fav1.value else _fav2.value)[index] ?: return onDone(false, "빈 슬롯")
        sendGetIr(item.id, onDone)
    }

    /** 슬롯 비우기 + 저장 */
    fun clearFavorite(which: Int, index: Int): Pair<Boolean, String> {
        val flow = if (which == 1) _fav1 else _fav2
        val list = flow.value.toMutableList()
        if (index !in list.indices) return false to "잘못된 슬롯"
        if (list[index] == null) return false to "이미 빈 슬롯"
        list[index] = null
        flow.value = list
        persist(which, list)
        return true to "슬롯 비움"
    }

    /** 슬롯 이름 바꾸기 + 저장 */
    fun renameFavorite(which: Int, index: Int, newName: String): Pair<Boolean, String> {
        val flow = if (which == 1) _fav1 else _fav2
        val list = flow.value.toMutableList()
        if (index !in list.indices) return false to "잘못된 슬롯"
        val cur = list[index] ?: return false to "빈 슬롯"
        list[index] = cur.copy(name = newName.ifBlank { cur.name })
        flow.value = list
        persist(which, list)
        return true to "이름 변경"
    }

    private fun persist(which: Int, list: List<FavItem?>) {
        viewModelScope.launch {
            FavoritesStore.save(applicationCtx, which, list)
        }
    }

    fun dismissRenameDialog() { _renameDialogTarget.value = null }

    fun renameSignal(newName: String, onDone: (Boolean, String) -> Unit = { _, _ -> }) {
        val id = _renameDialogTarget.value ?: return
        viewModelScope.launch {
            runCatching { ApiClient.api.renameSignal(id, RenameReq(newName)) }
                .onSuccess {
                    _renameDialogTarget.value = null
                    refresh()
                    onDone(true, "이름 변경 완료")
                }
                .onFailure { e -> onDone(false, e.message ?: "이름 변경 실패") }
        }
    }

    /** 기기에 저장 명령 */
    fun saveIr(
        mode: SaveMode,
        irSignal: String,
        signalName: String,
        onDone: (Boolean, String) -> Unit = { _, _ -> }
    ) {
        viewModelScope.launch {
            if (_connState.value !is ConnState.Connected) connect()

            // 이름은 @ 구분자 충돌 방지를 위해 URL-encode
//            val encodedName = try {
//                URLEncoder.encode(signalName.ifBlank { "IR" }, StandardCharsets.UTF_8.name())
//            } catch (e: Exception) {
//                // 혹시 모를 플랫폼 이슈 대비: 최소한의 대체
//                signalName.replace("@", "_").ifBlank { "IR" }
//            }

            val cmd = when (mode) {
                SaveMode.WIFI -> "[LDH_ARD]SAVEWIFI@$irSignal@$signalName"
                SaveMode.BLE  -> "[LDH_ARD]SAVEBLE@$irSignal@$signalName"
            }

            val res = TcpSession.send(cmd)
            if (res.isSuccess) onDone(true, "TX OK: $cmd")
            else {
                val msg = res.exceptionOrNull()?.message ?: "TX failed"
                _connState.value = ConnState.Failed(msg)
                onDone(false, msg)
            }
        }
    }

    /** 업로드 명령: [LDH_ARD]UPLOAD@ */
    fun upload(onDone: (Boolean, String) -> Unit = { _, _ -> }) {
        viewModelScope.launch {
            if (_connState.value !is ConnState.Connected) connect()
            val cmd = "[LDH_ARD]UPLOAD@"
            val res = TcpSession.send(cmd)
            if (res.isSuccess) onDone(true, "TX OK: $cmd")
            else {
                val msg = res.exceptionOrNull()?.message ?: "TX failed"
                _connState.value = ConnState.Failed(msg)
                onDone(false, msg)
            }
        }
    }
}
