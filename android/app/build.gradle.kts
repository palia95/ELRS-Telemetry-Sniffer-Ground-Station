// Minimal Compose app module for the ELRS Ghost telemetry viewer.
// Full project scaffolding (settings.gradle.kts, root build, gradle wrapper)
// is standard Android Studio "Empty Compose Activity"; only the app-specific
// bits are shown here.
plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
    id("org.jetbrains.kotlin.plugin.compose")
}

android {
    namespace = "com.elrs.ghost"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.elrs.ghost"
        minSdk = 26        // BLE peripheral-name filtering & modern APIs
        targetSdk = 34
        versionCode = 1
        versionName = "0.1"
    }
    buildFeatures { compose = true }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions { jvmTarget = "17" }
}

dependencies {
    implementation(platform("androidx.compose:compose-bom:2024.09.00"))
    implementation("androidx.compose.material3:material3")
    implementation("androidx.compose.ui:ui")
    implementation("androidx.activity:activity-compose:1.9.2")
    implementation("androidx.lifecycle:lifecycle-viewmodel-compose:2.8.6")
}
