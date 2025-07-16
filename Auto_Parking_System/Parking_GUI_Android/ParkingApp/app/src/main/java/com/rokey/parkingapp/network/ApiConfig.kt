import android.content.Context
import android.net.nsd.NsdManager
import android.net.nsd.NsdServiceInfo
import android.util.Log
import android.content.SharedPreferences

object ApiConfig {
    private const val SERVICE_TYPE = "_parkingapi._tcp."
    private const val DEFAULT_PORT = 8000
    var BASE_URL: String = ""
    var WS_BASE_URL: String = ""
    const val CONNECT_TIMEOUT = 10L
    const val READ_TIMEOUT = 10L
    const val WRITE_TIMEOUT = 10L
    const val MAX_RETRIES = 3
    const val INITIAL_RETRY_DELAY = 1000L
    const val MAX_RETRY_DELAY = 3000L
    const val RECONNECT_DELAY = 2000L
    const val SERVER_REACHABLE_TIMEOUT = 5000L
    const val MIN_RETRY_INTERVAL = 5000L

    private var nsdManager: NsdManager? = null
    private var discoveryListener: NsdManager.DiscoveryListener? = null
    private var resolveListener: NsdManager.ResolveListener? = null
    private var prefs: SharedPreferences? = null

    fun initializeDiscovery(context: Context) {
        nsdManager = context.getSystemService(Context.NSD_SERVICE) as NsdManager
        prefs = context.getSharedPreferences("server_prefs", Context.MODE_PRIVATE)
        
        resolveListener = object : NsdManager.ResolveListener {
            override fun onResolveFailed(serviceInfo: NsdServiceInfo, errorCode: Int) {
                Log.e("ApiConfig", "서비스 해석 실패: $errorCode")
            }

            override fun onServiceResolved(serviceInfo: NsdServiceInfo) {
                val host = serviceInfo.host.hostAddress
                val port = serviceInfo.port
                BASE_URL = "http://$host:$port"
                WS_BASE_URL = "ws://$host:$port"
                Log.d("ApiConfig", "서버 발견: $BASE_URL")
                
                // SharedPreferences에 저장
                prefs?.edit()?.apply {
                    putString("base_url", BASE_URL)
                    putString("ws_base_url", WS_BASE_URL)
                    apply()
                }
            }
        }

        discoveryListener = object : NsdManager.DiscoveryListener {
            override fun onStartDiscoveryFailed(serviceType: String, errorCode: Int) {
                Log.e("ApiConfig", "검색 시작 실패: $errorCode")
            }

            override fun onStopDiscoveryFailed(serviceType: String, errorCode: Int) {
                Log.e("ApiConfig", "검색 중지 실패: $errorCode")
            }

            override fun onDiscoveryStarted(serviceType: String) {
                Log.d("ApiConfig", "서비스 검색 시작")
            }

            override fun onDiscoveryStopped(serviceType: String) {
                Log.d("ApiConfig", "서비스 검색 중지")
            }

            override fun onServiceFound(serviceInfo: NsdServiceInfo) {
                Log.d("ApiConfig", "서비스 발견: ${serviceInfo.serviceName}")
                if (serviceInfo.serviceType == SERVICE_TYPE) {
                    nsdManager?.resolveService(serviceInfo, resolveListener)
                }
            }

            override fun onServiceLost(serviceInfo: NsdServiceInfo) {
                Log.e("ApiConfig", "서비스 연결 끊김: ${serviceInfo.serviceName}")
            }
        }
    }

    fun startDiscovery() {
        try {
            nsdManager?.discoverServices(
                SERVICE_TYPE,
                NsdManager.PROTOCOL_DNS_SD,
                discoveryListener
            )
            Log.d("ApiConfig", "서비스 검색 시작됨")
        } catch (e: Exception) {
            Log.e("ApiConfig", "서비스 검색 시작 실패", e)
        }
    }

    fun stopDiscovery() {
        try {
            nsdManager?.stopServiceDiscovery(discoveryListener)
        } catch (e: Exception) {
            Log.e("ApiConfig", "검색 중지 중 오류: ${e.message}")
        }
    }
} 