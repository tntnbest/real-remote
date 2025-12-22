package com.example.myremote2

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.viewModels
import androidx.compose.animation.animateColorAsState
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.ExperimentalFoundationApi
import androidx.compose.foundation.combinedClickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.grid.GridCells
import androidx.compose.foundation.lazy.grid.LazyVerticalGrid
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.myremote2.ui.theme.Myremote2Theme
import kotlinx.coroutines.launch

class MainActivity : ComponentActivity() {
    private val vm: SignalViewModel by viewModels()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContent {
            Myremote2Theme {
                Surface { RootScreen(vm) }
            }
        }
    }

    override fun onStart() {
        super.onStart()
        vm.connect() // 진입 시 TCP 로그인
    }

    override fun onStop() {
        super.onStop()
        vm.close()
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun RootScreen(vm: SignalViewModel) {
    var selectedTab by rememberSaveable { mutableStateOf(2) } // 기본 3번 탭(신호 저장소)
    val snackbarHost = remember { SnackbarHostState() }
    val scope = rememberCoroutineScope()
    val conn by vm.connState.collectAsState()

    Scaffold(
        topBar = {
            TopAppBar(
                title = {
                    Text(
                        when (selectedTab) {
                            0 -> "내 리모콘 1"
                            1 -> "내 리모콘 2"
                            else -> "신호 저장소"
                        }
                    )
                },
                actions = {
                    if (selectedTab == 2) {
                        Text(
                            when (conn) {
                                ConnState.Idle -> "연결 안됨"
                                ConnState.Connecting -> "연결 중…"
                                ConnState.Connected -> "연결됨"
                                is ConnState.Failed -> "오류"
                            }
                        )
                        Spacer(Modifier.width(12.dp))
                        TextButton(onClick = { vm.refresh() }) { Text("새로고침") }

                        Spacer(Modifier.width(8.dp))

                        // 초록색 업로드 버튼
                        Button(
                            onClick = {
                                vm.upload { ok, msg ->
                                    scope.launch { snackbarHost.showSnackbar(if (ok) "업로드 전송 완료" else "실패: $msg") }
                                }
                            },
                            colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.primary)
                        ) {
                            Text("업로드", color = MaterialTheme.colorScheme.onPrimary)
                        }
                    }
                }
            )
        },
        bottomBar = {
            NavigationBar {
                NavigationBarItem(
                    selected = selectedTab == 0,
                    onClick = { selectedTab = 0 },
                    icon = { Text("1") },
                    label = { Text("즐겨찾기1") }
                )
                NavigationBarItem(
                    selected = selectedTab == 1,
                    onClick = { selectedTab = 1 },
                    icon = { Text("2") },
                    label = { Text("즐겨찾기2") }
                )
                NavigationBarItem(
                    selected = selectedTab == 2,
                    onClick = { selectedTab = 2 },
                    icon = { Text("3") },
                    label = { Text("신호 저장소") }
                )
            }
        },
        snackbarHost = { SnackbarHost(snackbarHost) }
    ) { inner ->
        when (selectedTab) {
            0 -> FavoritesTab(which = 1, vm = vm, modifier = Modifier.padding(inner)) { msg ->
                scope.launch { snackbarHost.showSnackbar(msg) }
            }
            1 -> FavoritesTab(which = 2, vm = vm, modifier = Modifier.padding(inner)) { msg ->
                scope.launch { snackbarHost.showSnackbar(msg) }
            }
            2 -> SignalsTab(
                vm = vm,
                modifier = Modifier.padding(inner),
                onToast = { msg -> scope.launch { snackbarHost.showSnackbar(msg) } }
            )
        }
    }
}

@OptIn(ExperimentalFoundationApi::class)
@Composable
fun FavoritesTab(
    which: Int,                  // 1 or 2
    vm: SignalViewModel,
    modifier: Modifier = Modifier,
    onToast: (String) -> Unit = {}
) {
    val slots by (if (which == 1) vm.fav1 else vm.fav2).collectAsState()

    var menuIndex by remember { mutableStateOf<Int?>(null) }
    var renameIndex by remember { mutableStateOf<Int?>(null) }
    var renameText by remember { mutableStateOf("") }

    val currentSlot = menuIndex?.let { idx -> slots.getOrNull(idx) }
    if (menuIndex != null) {
        AlertDialog(
            onDismissRequest = { menuIndex = null },
            title = { Text("버튼 옵션") },
            text = { Text(currentSlot?.name ?: "빈 버튼") },
            confirmButton = {
                TextButton(onClick = {
                    val idx = menuIndex!!
                    if (currentSlot == null) {
                        onToast("빈 버튼")
                        menuIndex = null
                    } else {
                        renameIndex = idx
                        renameText = currentSlot.name
                        menuIndex = null
                    }
                }) { Text("이름 바꾸기") }
            },
            dismissButton = {
                Row {
                    TextButton(onClick = {
                        val idx = menuIndex!!
                        val (ok, msg) = vm.clearFavorite(which, idx)
                        onToast(if (ok) "버튼 비움" else msg)
                        menuIndex = null
                    }) { Text("비우기") }
                    Spacer(Modifier.width(8.dp))
                    TextButton(onClick = { menuIndex = null }) { Text("닫기") }
                }
            }
        )
    }

    if (renameIndex != null) {
        AlertDialog(
            onDismissRequest = { renameIndex = null },
            title = { Text("이름 바꾸기") },
            text = {
                OutlinedTextField(
                    value = renameText,
                    onValueChange = { renameText = it },
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth()
                )
            },
            confirmButton = {
                TextButton(onClick = {
                    val idx = renameIndex!!
                    val (ok, msg) = vm.renameFavorite(which, idx, renameText)
                    onToast(if (ok) "이름 변경" else msg)
                    renameIndex = null
                }) { Text("저장") }
            },
            dismissButton = { TextButton(onClick = { renameIndex = null }) { Text("취소") } }
        )
    }

    LazyVerticalGrid(
        columns = GridCells.Fixed(4),
        modifier = modifier.fillMaxSize(),
        contentPadding = PaddingValues(12.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
        horizontalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        items(slots.size) { index ->
            val slot = slots[index]
            val isFilled = slot != null

            // 채워진 슬롯이면 연한 컨테이너 색, 아니면 서페이스 바리언트 + 얇은 테두리
            val bg by animateColorAsState(
                targetValue = if (isFilled)
                    MaterialTheme.colorScheme.secondaryContainer
                else
                    MaterialTheme.colorScheme.surfaceVariant,
                label = "slotBg"
            )
            val fg by animateColorAsState(
                targetValue = if (isFilled)
                    MaterialTheme.colorScheme.onSecondaryContainer
                else
                    MaterialTheme.colorScheme.onSurfaceVariant,
                label = "slotFg"
            )
            val border = if (isFilled) null
            else BorderStroke(1.dp, MaterialTheme.colorScheme.outline)

            Surface(
                color = bg,
                contentColor = fg,
                shape = MaterialTheme.shapes.medium,
                tonalElevation = 0.dp,
                border = border,
                modifier = Modifier
                    .fillMaxWidth(0.9f)
                    .aspectRatio(1f)
                    .combinedClickable(
                        onClick = {
                            if (slot != null) {
                                vm.sendFavorite(which, index) { ok, msg ->
                                    onToast(if (ok) "전송: ${slot.name}" else "실패: $msg")
                                }
                            } else {
                                onToast("빈 버튼")
                            }
                        },
                        onLongClick = { menuIndex = index }
                    )
            ) {
                Box(Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
                    Text(
                        text = slot?.name ?: "", // 빈 버튼 이름
                        maxLines = 1,
                        overflow = TextOverflow.Ellipsis,
                        fontSize = 12.sp
                    )
                }
            }
        }
    }
}

/** 3번 탭: 신호 저장소 */
@Composable
fun SignalsTab(
    vm: SignalViewModel,
    modifier: Modifier = Modifier,
    onToast: (String) -> Unit = {}
) {
    val state by vm.uiState.collectAsState()

    val renameTargetId by vm.renameDialogTarget.collectAsState()
    var newName by rememberSaveable { mutableStateOf("") }

    if (renameTargetId != null) {
        AlertDialog(
            onDismissRequest = {
                vm.dismissRenameDialog()
                newName = ""
            },
            title = { Text("신호 이름 지정") },
            text = {
                OutlinedTextField(
                    value = newName,
                    onValueChange = { newName = it },
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth(),
                    placeholder = { Text("예: TV_전원") }
                )
            },
            confirmButton = {
                TextButton(onClick = {
                    val name = newName.trim()
                    if (name.isNotEmpty()) {
                        vm.renameSignal(name) { ok, msg ->
                            onToast(if (ok) "이름 저장 완료" else "실패: $msg")
                        }
                        newName = ""
                    } else {
                        onToast("이름을 입력하세요")
                    }
                }) { Text("저장") }
            },
            dismissButton = {
                TextButton(onClick = {
                    vm.dismissRenameDialog()
                    newName = ""
                }) { Text("취소") }
            }
        )
    }

    // 즐겨찾기 선택 다이얼로그 제어
    var pickFor by remember { mutableStateOf<Signal?>(null) }
    var downloadFor by remember { mutableStateOf<Signal?>(null) }

    if (pickFor != null) {
        AlertDialog(
            onDismissRequest = { pickFor = null },
            title = { Text("저장할 곳 선택") },
            // text = { Text("즐겨찾기 위치를 선택하세요") }, // 필요하면 설명 추가
            confirmButton = {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.Center
                ) {
                    TextButton(onClick = {
                        pickFor?.let { sig ->
                            val (ok, msg) = vm.addFavorite(1, sig)
                            onToast(if (ok) "즐겨찾기1: $msg" else msg)
                        }
                        pickFor = null
                    }) { Text("즐겨찾기 1") }

                    Spacer(Modifier.width(8.dp))

                    TextButton(onClick = {
                        pickFor?.let { sig ->
                            val (ok, msg) = vm.addFavorite(2, sig)
                            onToast(if (ok) "즐겨찾기2: $msg" else msg)
                        }
                        pickFor = null
                    }) { Text("즐겨찾기 2") }

                    Spacer(Modifier.width(8.dp))

                    TextButton(onClick = { pickFor = null }) { Text("닫기") }
                }
            },
            dismissButton = {}
        )
    }

    if (downloadFor != null) {
        AlertDialog(
            onDismissRequest = { downloadFor = null },
            title = { Text("다운로드 방식 선택") },
            confirmButton = {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.Center
                ) {
                    TextButton(onClick = {
                        downloadFor?.let { sig ->
                            vm.saveIr(SaveMode.WIFI, sig.ir_signal, sig.signal_name) { ok, msg ->
                                onToast(if (ok) "Wi‑Fi 저장 전송 완료" else "실패: $msg")
                            }
                        }
                        downloadFor = null
                    }) { Text("와이파이") }

                    Spacer(Modifier.width(8.dp))

                    TextButton(onClick = {
                        downloadFor?.let { sig ->
                            vm.saveIr(SaveMode.BLE, sig.ir_signal, sig.signal_name) { ok, msg ->
                                onToast(if (ok) "BLE 저장 전송 완료" else "실패: $msg")
                            }
                        }
                        downloadFor = null
                    }) { Text("블루투스") }

                    Spacer(Modifier.width(8.dp))

                    TextButton(onClick = { downloadFor = null }) { Text("닫기") }
                }
            },
            dismissButton = {}
        )
    }

    Box(modifier.fillMaxSize()) {
        when (val s = state) {
            is SignalsUiState.Loading -> CircularProgressIndicator(Modifier.align(Alignment.Center))
            is SignalsUiState.Error -> {
                Column(
                    modifier = Modifier.align(Alignment.Center).padding(16.dp),
                    horizontalAlignment = Alignment.CenterHorizontally
                ) {
                    Text("불러오기 실패: ${s.message}")
                    Spacer(Modifier.height(8.dp))
                    Button(onClick = { vm.refresh() }) { Text("다시 시도") }
                }
            }
            is SignalsUiState.Success -> {
                if (s.items.isEmpty()) {
                    Text("데이터가 없습니다.", modifier = Modifier.align(Alignment.Center))
                } else {
                    LazyColumn(
                        modifier = Modifier.fillMaxSize(),
                        contentPadding = PaddingValues(12.dp),
                        verticalArrangement = Arrangement.spacedBy(8.dp)
                    ) {
                        items(s.items) { item ->
                            SignalRow(
                                item = item,
                                onSend = { id ->
                                    vm.sendGetIr(id) { ok, msg ->
                                        onToast(if (ok) "전송 완료: $id" else "실패: $msg")
                                    }
                                },
                                onFavorite = { sig -> pickFor = sig },
                                onDownload = { sig -> downloadFor = sig }
                            )
                        }
                    }
                }
            }
        }
    }
}


@Composable
fun SignalRow(
    item: Signal,
    onSend: (Int) -> Unit,
    onFavorite: (Signal) -> Unit,
    onDownload: (Signal) -> Unit,
    showId: Boolean = false   // ← 기본값은 숨김
) {
    ElevatedCard(Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp)) {
            if (showId) {
                Text(
                    "ID: ${item.id}",
                    style = MaterialTheme.typography.labelSmall,
                    color = MaterialTheme.colorScheme.outline
                )
                Spacer(Modifier.height(4.dp))
            }
            Text(item.signal_name, style = MaterialTheme.typography.titleMedium, maxLines = 1)
            Spacer(Modifier.height(2.dp))
            Text("${item.uploader}", style = MaterialTheme.typography.bodyMedium, maxLines = 1)
            Spacer(Modifier.height(8.dp))
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                OutlinedButton(onClick = { onFavorite(item) }) { Text("즐겨찾기") }
                OutlinedButton(onClick = { onDownload(item) }) { Text("다운로드") }
                Button(onClick = { onSend(item.id) }) { Text("전송") }
            }
        }
    }
}