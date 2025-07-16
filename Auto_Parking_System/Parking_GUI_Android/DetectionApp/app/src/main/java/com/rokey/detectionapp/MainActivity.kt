package com.rokey.detectionapp

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.fragment.app.Fragment
import com.rokey.detectionapp.databinding.ActivityMainBinding
import com.rokey.detectionapp.ui.home.HomeFragment
import com.rokey.detectionapp.ui.detection.DetectionFragment

class MainActivity : AppCompatActivity() {
    
    private lateinit var binding: ActivityMainBinding
    
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        
        // 기본 프래그먼트 설정
        if (savedInstanceState == null) {
            loadFragment(HomeFragment())
        }
        
        // 바텀 네비게이션 설정
        binding.bottomNavigation.setOnItemSelectedListener { item ->
            when (item.itemId) {
                R.id.navigation_home -> loadFragment(HomeFragment())
                R.id.navigation_detection -> loadFragment(DetectionFragment())
                else -> false
            }
        }
    }
    
    private fun loadFragment(fragment: Fragment): Boolean {
        supportFragmentManager.beginTransaction()
            .replace(R.id.fragment_container, fragment)
            .commit()
        return true
    }
} 