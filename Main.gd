extends Node

# load the visualiser library
onready var data = preload("res://build/OpenHMDVisualiser.gdns").new()

func alert(text: String, title: String='Message') -> void:
	var dialog = AcceptDialog.new()
	dialog.dialog_text = text
	dialog.window_title = title
	dialog.connect('modal_closed', dialog, 'queue_free')
	add_child(dialog)
	dialog.popup_centered()
	
func _on_PlayPauseButton_pressed():
	pass

func _on_LoadButton_pressed():
	var dialog = FileDialog.new()
	dialog.access = FileDialog.ACCESS_FILESYSTEM
	dialog.mode = FileDialog.MODE_OPEN_FILE
	dialog.add_filter("*.mkv ; MKV OpenHMD recording")
	dialog.resizable = true
	dialog.window_title = "Select Recording File"
	dialog.connect("file_selected", self, "_on_file_selected")
	add_child(dialog)
	dialog.popup_centered_ratio()

func _on_file_selected(path):
	if data.load_file(path):
		alert("Loaded file" + path)
