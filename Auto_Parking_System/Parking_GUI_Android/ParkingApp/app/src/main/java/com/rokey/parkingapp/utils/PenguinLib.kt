package com.rokey.parkingapp.utils

class PenguinLib {
    companion object {
        init {
            System.loadLibrary("penguin")
        }
    }

    external fun initialize(): Boolean
    external fun processImage(imageData: String): String
    external fun cleanup()
} 