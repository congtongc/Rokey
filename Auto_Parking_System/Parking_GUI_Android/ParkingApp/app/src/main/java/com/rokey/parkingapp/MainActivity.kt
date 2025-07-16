package com.rokey.parkingapp

import android.app.AlertDialog
import android.content.Context
import android.content.pm.PackageManager
import android.net.nsd.NsdManager
import android.net.nsd.NsdServiceInfo
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.EditText
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import androidx.navigation.NavController
import androidx.navigation.fragment.NavHostFragment
import androidx.navigation.ui.AppBarConfiguration
import androidx.navigation.ui.setupActionBarWithNavController
import androidx.navigation.ui.setupWithNavController
import com.google.android.material.bottomnavigation.BottomNavigationView
import com.google.android.material.snackbar.Snackbar
import com.rokey.parkingapp.R
import com.rokey.parkingapp.databinding.ActivityMainBinding
import com.rokey.parkingapp.network.ApiClient
import com.rokey.parkingapp.network.ServerConfig
import com.rokey.parkingapp.network.WebSocketManager
import com.rokey.parkingapp.network.ConnectionState
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.cancel
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

class MainActivity : AppCompatActivity() {
    
    private lateinit var binding: ActivityMainBinding
    private lateinit var navController: NavController
    private val LOCATION_PERMISSION_REQUEST_CODE = 1001
    private val TAG = "MainActivity"
    private val apiClient by lazy { ApiClient.getInstance() }
    private val scope = CoroutineScope(Dispatchers.Main)
    private var isFinishing = false
    private var currentSnackbar: Snackbar? = null
    
    // 기본 서버 목록 추가
    private val DEFAULT_SERVERS = listOf(
        "http://172.16.0.178:8000",
        "http://10.0.2.2:8000",
        "http://192.168.0.12:8000",
        "http://localhost:8000",
        "http://127.0.0.1:8000"
    )
    
    private val DEFAULT_ENDPOINTS = listOf(
        "/parking/status",
        "/status",
        "/",
        "/api/parking/status"
    )
    
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        
        // ApiClient 초기화를 가장 먼저 수행
        ApiClient.initialize(applicationContext)
        
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        // Toolbar 설정
        setSupportActionBar(binding.toolbar)
        
        setupNavigation()
        setupConnectionMonitoring()
        
        // 위치 권한 체크 및 요청
        checkLocationPermission()
    }

    private fun setupNavigation() {
        // NavHostFragment 찾기
        val navHostFragment = supportFragmentManager
            .findFragmentById(R.id.nav_host_fragment_activity_main) as NavHostFragment
        navController = navHostFragment.navController
        
        // 네비게이션 설정
        val appBarConfiguration = AppBarConfiguration(
            setOf(
                R.id.navigation_home, R.id.navigation_parking, R.id.navigation_exit
            )
        )
        
        setupActionBarWithNavController(navController, appBarConfiguration)
        binding.navView.setupWithNavController(navController)
    }

    override fun onSupportNavigateUp(): Boolean {
        return navController.navigateUp() || super.onSupportNavigateUp()
    }

    private fun checkLocationPermission() {
        if (ContextCompat.checkSelfPermission(
                this,
                android.Manifest.permission.ACCESS_FINE_LOCATION
            ) != PackageManager.PERMISSION_GRANTED
        ) {
            ActivityCompat.requestPermissions(
                this,
                arrayOf(android.Manifest.permission.ACCESS_FINE_LOCATION),
                LOCATION_PERMISSION_REQUEST_CODE
            )
        } else {
            // 저장된 서버 주소가 있는지 확인
            val prefs = getSharedPreferences("server_prefs", Context.MODE_PRIVATE)
            val savedBaseUrl = prefs.getString("base_url", null)
            val savedWsUrl = prefs.getString("ws_base_url", null)
            
            if (savedBaseUrl != null && savedWsUrl != null) {
                Log.d(TAG, "저장된 서버 주소 사용: $savedBaseUrl")
                lifecycleScope.launch {
                    try {
                        ApiClient.updateServerUrl(this@MainActivity, savedBaseUrl)
                    } catch (e: Exception) {
                        Log.e(TAG, "저장된 서버 연결 실패", e)
                        if (isActive()) {
                            withContext(Dispatchers.Main) {
                                improvedAutoDetectServer()
                            }
                        }
                    }
                }
            } else {
                // 자동 서버 탐색
                improvedAutoDetectServer()
            }
        }
    }
    
    private fun showServerConfigDialog() {
        if (!isActive()) return
        
        try {
            val serverConfig = ServerConfig.getInstance(this)
            val currentAddress = "${serverConfig.serverIp}:${serverConfig.serverPort}"

            val dialogView = layoutInflater.inflate(R.layout.dialog_server_config, null)
            val input = dialogView.findViewById<EditText>(R.id.etServerAddress)
            input.setText(currentAddress)

            val dialog = AlertDialog.Builder(this)
                .setTitle("서버 설정")
                .setMessage("서버 IP 주소와 포트를 입력하세요")
                .setView(dialogView)
                .setCancelable(false)
                .setPositiveButton("확인", null)
                .setNegativeButton("재시도", null)
                .create()

            dialog.setOnShowListener {
                dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener {
                    val address = input.text.toString()
                    if (address.isNotEmpty()) {
                        val httpUrl = "http://$address"
                        lifecycleScope.launch {
                            try {
                                ApiClient.updateServerUrl(this@MainActivity, httpUrl)
                                dialog.dismiss()
                            } catch (e: Exception) {
                                Log.e(TAG, "서버 연결 실패", e)
                                Toast.makeText(this@MainActivity, "서버 연결 실패: ${e.message}", Toast.LENGTH_LONG).show()
                            }
                        }
                    }
                }
                dialog.getButton(AlertDialog.BUTTON_NEGATIVE).setOnClickListener {
                    dialog.dismiss()
                    improvedAutoDetectServer()
                }
            }

            if (isActive()) {
                dialog.show()
            }
        } catch (e: Exception) {
            Log.e(TAG, "다이얼로그 표시 오류", e)
            if (isActive()) {
                Toast.makeText(this, "서버 설정 화면을 표시할 수 없습니다.", Toast.LENGTH_LONG).show()
            }
        }
    }

    private fun isActive(): Boolean {
        return !isFinishing && !isDestroyed && lifecycle.currentState.isAtLeast(Lifecycle.State.RESUMED)
    }

    override fun onDestroy() {
        super.onDestroy()
        isFinishing = true
        scope.cancel()
        WebSocketManager.disconnect()
    }

    private fun improvedAutoDetectServer() {
        if (!isActive()) return
        
        lifecycleScope.launch {
            try {
                for (serverUrl in DEFAULT_SERVERS) {
                    if (!isActive()) return@launch
                    
                    for (endpoint in DEFAULT_ENDPOINTS) {
                        if (!isActive()) return@launch
                        
                        try {
                            ApiClient.updateServerUrl(this@MainActivity, serverUrl)
                            
                            val response = apiClient.getParkingStatus()
                            if (response.success) {
                                val prefs = getSharedPreferences("server_prefs", Context.MODE_PRIVATE)
                                prefs.edit()
                                    .putString("base_url", serverUrl)
                                    .putString("ws_base_url", serverUrl.replace("http", "ws"))
                                    .apply()
                                
                                if (isActive()) {
                                    withContext(Dispatchers.Main) {
                                        Toast.makeText(
                                            this@MainActivity, 
                                            "서버 자동 연결 성공: $serverUrl", 
                                            Toast.LENGTH_LONG
                                        ).show()
                                    }
                                }
                                return@launch
                            }
                        } catch (e: Exception) {
                            Log.e(TAG, "서버 연결 시도 실패: $serverUrl, 엔드포인트: $endpoint", e)
                        }
                    }
                }
                
                // 모든 시도 실패 시
                if (isActive()) {
                    withContext(Dispatchers.Main) {
                        showServerConfigDialog()
                    }
                }
            } catch (e: Exception) {
                Log.e(TAG, "서버 자동 탐색 실패", e)
                if (isActive()) {
                    withContext(Dispatchers.Main) {
                        showServerConfigDialog()
                    }
                }
            }
        }
    }
    
    private fun setupConnectionMonitoring() {
        lifecycleScope.launch {
            lifecycle.repeatOnLifecycle(Lifecycle.State.STARTED) {
                launch {
                    ConnectionState.isConnected.collectLatest { isConnected ->
                        if (isActive()) {
                            updateConnectionUI(isConnected)
                        }
                    }
                }
                
                launch {
                    ConnectionState.lastError.collectLatest { error ->
                        if (isActive()) {
                            error?.let { showError(it) }
                        }
                    }
                }
            }
        }
    }
    
    private fun updateConnectionUI(isConnected: Boolean) {
        if (!isActive()) return
        
        currentSnackbar?.dismiss()
        
        if (!isConnected) {
            currentSnackbar = Snackbar.make(
                binding.root,
                "서버 연결이 끊어졌습니다. 재연결 중...",
                Snackbar.LENGTH_INDEFINITE
            ).apply { show() }
        }
    }
    
    private fun showError(error: String) {
        if (!isActive()) return
        
        currentSnackbar?.dismiss()
        currentSnackbar = Snackbar.make(
            binding.root,
            error,
            Snackbar.LENGTH_LONG
        ).apply { show() }
    }
} 