data class OcrResult(
    val car_plate: String,
    val type: String,
    val confidence: Double,
    val timestamp: String?
) 