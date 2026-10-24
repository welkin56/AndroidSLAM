package com.example.slamcapture

import android.app.Activity
import android.app.Application
import android.content.res.Configuration
import androidx.core.view.WindowCompat
import com.google.android.material.color.DynamicColors

/**
 * Android 12+ 从壁纸取色；低版本留在主题里的青色方案。
 * 必须挂在 Application 上：动态色要在 Activity 的 onCreate 之前应用。
 */
class SlamApp : Application() {
    override fun onCreate() {
        super.onCreate()
        DynamicColors.applyToActivitiesIfAvailable(this)
    }
}

/** 系统栏图标明暗跟随主题；API 26 的导航栏没有主题属性，只能运行时设。 */
fun Activity.applySystemBarAppearance() {
    val night = (resources.configuration.uiMode and Configuration.UI_MODE_NIGHT_MASK) ==
        Configuration.UI_MODE_NIGHT_YES
    WindowCompat.getInsetsController(window, window.decorView).apply {
        isAppearanceLightStatusBars = !night
        isAppearanceLightNavigationBars = !night
    }
}
