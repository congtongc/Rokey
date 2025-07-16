package com.rokey.parkingapp.network

import android.content.Context
import android.content.SharedPreferences

class ServerConfig private constructor(context: Context) {
    private val prefs: SharedPreferences = context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE)

    companion object {
        private const val PREF_NAME = "server_config"
        private const val KEY_SERVER_IP = "server_ip"
        private const val KEY_SERVER_PORT = "server_port"
        private const val DEFAULT_PORT = "8000"
        
        @Volatile
        private var instance: ServerConfig? = null

        fun getInstance(context: Context): ServerConfig {
            return instance ?: synchronized(this) {
                instance ?: ServerConfig(context.applicationContext).also { instance = it }
            }
        }
    }

    var serverIp: String
        get() = prefs.getString(KEY_SERVER_IP, "172.16.0.178") ?: "172.16.0.178"
        set(value) = prefs.edit().putString(KEY_SERVER_IP, value).apply()

    var serverPort: String
        get() = prefs.getString(KEY_SERVER_PORT, DEFAULT_PORT) ?: DEFAULT_PORT
        set(value) = prefs.edit().putString(KEY_SERVER_PORT, value).apply()

    fun getServerUrl(): String {
        return "http://$serverIp:$serverPort"
    }

    fun saveServerAddress(address: String) {
        val parts = address.split(":")
        if (parts.size == 2) {
            serverIp = parts[0]
            serverPort = parts[1]
        }
    }
} 