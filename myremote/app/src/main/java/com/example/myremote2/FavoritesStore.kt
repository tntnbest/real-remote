package com.example.myremote2

import android.content.Context
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.stringPreferencesKey
import androidx.datastore.preferences.preferencesDataStore
import com.squareup.moshi.Moshi
import com.squareup.moshi.kotlin.reflect.KotlinJsonAdapterFactory
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map

data class FavSlot(val idx: Int, val item: FavItem)
data class FavSave(val slots: List<FavSlot>)

private val Context.dataStore by preferencesDataStore("favorites_ds")

object FavoritesStore {
    private val KEY_FAV1 = stringPreferencesKey("fav1_json")
    private val KEY_FAV2 = stringPreferencesKey("fav2_json")

    private val moshi = Moshi.Builder()
        .addLast(KotlinJsonAdapterFactory())
        .build()
    private val saveAdapter = moshi.adapter(FavSave::class.java)

    /** 저장값 Flow → 24칸 리스트로 복원 */
    fun flow(context: Context, which: Int, capacity: Int): Flow<List<FavItem?>> =
        context.dataStore.data.map { prefs ->
            val key = if (which == 1) KEY_FAV1 else KEY_FAV2
            val json = prefs[key] ?: return@map List(capacity) { null }
            val save = runCatching { saveAdapter.fromJson(json) }.getOrNull()
            val list = MutableList<FavItem?>(capacity) { null }
            save?.slots?.forEach { s ->
                if (s.idx in 0 until capacity) list[s.idx] = s.item
            }
            list
        }

    /** 현재 24칸 리스트를 JSON으로 저장 */
    suspend fun save(context: Context, which: Int, list: List<FavItem?>) {
        val key = if (which == 1) KEY_FAV1 else KEY_FAV2
        val filled = list.mapIndexedNotNull { idx, item ->
            if (item != null) FavSlot(idx, item) else null
        }
        val json = saveAdapter.toJson(FavSave(filled))
        context.dataStore.edit { it[key] = json }
    }
}
