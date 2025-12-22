package com.example.myremote2

import okhttp3.OkHttpClient
import okhttp3.logging.HttpLoggingInterceptor
import retrofit2.Retrofit
import retrofit2.converter.moshi.MoshiConverterFactory
import retrofit2.http.GET
import retrofit2.http.PUT
import retrofit2.http.Path
import retrofit2.http.Body
import com.squareup.moshi.Moshi
import com.squareup.moshi.kotlin.reflect.KotlinJsonAdapterFactory
import java.util.concurrent.TimeUnit

//private const val BASE_URL = "http://10.0.2.2:3000/"
//private const val BASE_URL = "http://192.168.200.180:3000/"
private const val BASE_URL = "http://10.10.14.84:3000/"

// 서버 응답 필드에 맞춘 데이터 클래스
data class Signal(
    val id: Int,
    val signal_name: String,
    val ir_signal: String,
    val uploader: String
)

// 이름 변경 요청 바디
data class RenameReq(val signal_name: String)

interface IotApi {
    @GET("signals")
    suspend fun getSignals(): List<Signal>

    @PUT("signals/{id}/name")
    suspend fun renameSignal(
        @Path("id") id: Int,
        @Body body: RenameReq
    ): Signal
}

object ApiClient {
    private val http = OkHttpClient.Builder()
        .addInterceptor(HttpLoggingInterceptor().apply {
            level = HttpLoggingInterceptor.Level.BODY // 디버깅에 유용
        })
        // 타임아웃 기본값 보강 (필요 시 조정)
        .connectTimeout(5, TimeUnit.SECONDS)
        .readTimeout(10, TimeUnit.SECONDS)
        .writeTimeout(10, TimeUnit.SECONDS)
        .build()

    private val moshi: Moshi = Moshi.Builder()
        .addLast(KotlinJsonAdapterFactory())
        .build()

    val api: IotApi by lazy {
        Retrofit.Builder()
            .baseUrl(BASE_URL)
            .client(http)
            .addConverterFactory(MoshiConverterFactory.create(moshi))
            .build()
            .create(IotApi::class.java)
    }
}