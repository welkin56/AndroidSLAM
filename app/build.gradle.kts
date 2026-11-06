import java.util.Properties

plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

// 签名材料不进 git（见 .gitignore）。没有这个文件时 release 仍然能构建，
// 只是产出 unsigned 包——自己用 debug 包时不必配。
val keystoreProps = Properties().apply {
    val f = rootProject.file("keystore.properties")
    if (f.isFile) f.inputStream().use { load(it) }
}
val releaseStorePath: String? = keystoreProps.getProperty("storeFile")

android {
    namespace = "com.example.slamcapture"
    compileSdk = 34
    ndkVersion = "26.1.10909125"

    defaultConfig {
        applicationId = "com.example.slamcapture"
        minSdk = 26
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"

        ndk {
            abiFilters += "arm64-v8a"    // 只打 64 位 ARM：APK 减半，也避免 ABI 混装
        }
    }

    // 必须在 buildTypes 之前：buildTypes 里要用 getByName 取这个名字。
    signingConfigs {
        if (releaseStorePath != null) {
            create("release") {
                storeFile = rootProject.file(releaseStorePath)
                storePassword = keystoreProps.getProperty("storePassword")
                keyAlias = keystoreProps.getProperty("keyAlias")
                keyPassword = keystoreProps.getProperty("keyPassword")
            }
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            if (releaseStorePath != null) {
                signingConfig = signingConfigs.getByName("release")
            }
        }
    }

    sourceSets {
        // 词袋只进 release：debug 继续用 tools/push-vocab.sh 推的那个文件，
        // 免得每改一行代码都重新打 139 MB。目录不存在时 AGP 直接跳过。
        getByName("release") {
            assets.srcDir("src/release/assets")
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }

    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")   // native 工程的入口
        }
    }
}

dependencies {
    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.appcompat:appcompat:1.7.0")
    implementation("com.google.android.material:material:1.12.0")
}

// 打 release 时必须已经放了词袋，否则会静默产出一个「装完提示去跑 push 脚本」的包。
// 用 matching 而不是 named：任务名随 AGP 版本变，匹配不到时不至于让整份脚本解析失败。
val releaseVocab = file("src/release/assets/ORBvoc.txt")
tasks.matching { it.name.startsWith("merge") && it.name.endsWith("ReleaseAssets") }
    .configureEach {
        inputs.file(releaseVocab).optional()
        doFirst {
            if (!releaseVocab.isFile || releaseVocab.length() < 50_000_000L) {
                throw GradleException(
                    "release 缺词袋：${releaseVocab.path}\n" +
                        "先跑 tools/prepare-release-vocab.sh 把 ORBvoc.txt 复制进来（139 MB，不进 git）。"
                )
            }
        }
    }
