from pathlib import Path

# 1. Keep UI in PTZ mode after START PTZ
p = Path("web/index.html")
s = p.read_text()

s = s.replace(
    "$('controlMode').value = 'manual';\n    if ($('auto')) $('auto').checked = false;\n    syncControlModeUi();\n    saveSettings();\n    disconnectWs();\n    const tr = await apiGetJson('/api/tracker/state');",
    "$('controlMode').value = 'ptz';\n    if ($('auto')) $('auto').checked = false;\n    syncControlModeUi();\n    saveSettings();\n    disconnectWs();\n    const tr = await apiGetJson('/api/tracker/state');"
)

s = s.replace(
    "$('touchControls').classList.toggle('disabled', isAuto);",
    "$('touchControls').classList.toggle('disabled', isAuto || isPtz);"
)

s = s.replace(
    "Boxes: <code>DET=yellow TRACK=cyan SEL=green</code>",
    "Boxes: <code>DET=yellow TRACK=cyan SEL=red</code>"
)

p.write_text(s)
print("patched web/index.html")


# 2. Log UART TX from bridge
p = Path("web/ws_uart_bridge.cpp")
s = p.read_text()

s = s.replace(
'''        if (!last_joy_line.empty()) {
          bridge_->send_uart_line(last_joy_line);
        }''',
'''        if (!last_joy_line.empty()) {
          std::cerr << "UART_TX " << last_joy_line << "\\n";
          bridge_->send_uart_line(last_joy_line);
        }'''
)

s = s.replace(
'''              bridge_->send_uart_line(line);  // priority: send immediately''',
'''              std::cerr << "UART_TX " << line << "\\n";
              bridge_->send_uart_line(line);  // priority: send immediately'''
)

p.write_text(s)
print("patched web/ws_uart_bridge.cpp")
