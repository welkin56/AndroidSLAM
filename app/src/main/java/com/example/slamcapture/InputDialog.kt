package com.example.slamcapture

import android.app.Activity
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import com.google.android.material.textfield.TextInputEditText

/**
 * 单行文字输入弹窗。
 *
 * 空输入、取消、点外面都交给调用方处理：[onOk] 只带原文，去空和兜底由调用方决定；
 * 没点「确定」就关掉一律走 [onCancel] —— 不这么写，返回键关掉弹窗时调用方永远等不到回调。
 */
fun Activity.textInputDialog(
    title: String,
    initial: String,
    onCancel: () -> Unit = {},
    onOk: (String) -> Unit,
) {
    val v = layoutInflater.inflate(R.layout.dialog_text_input, null)
    val input = v.findViewById<TextInputEditText>(R.id.input)
    input.setText(initial)
    input.setSelection(initial.length)
    var confirmed = false
    val dlg = MaterialAlertDialogBuilder(this)
        .setTitle(title)
        .setView(v)
        .setPositiveButton("确定") { _, _ ->
            confirmed = true
            onOk(input.text?.toString()?.trim().orEmpty())
        }
        .setNegativeButton("取消", null)
        .create()
    dlg.setOnDismissListener { if (!confirmed) onCancel() }
    dlg.show()
}
