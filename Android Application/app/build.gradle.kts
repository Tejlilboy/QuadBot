plugins {
    id("com.android.application")
}

android {
    namespace = "com.quadbot.controller"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.quadbot.controller"
        minSdk = 29          // Android 10: needed for joining the robot's WiFi from inside the app
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
}

// No third-party libraries: the app is a thin shell around the control page.
dependencies {
}
