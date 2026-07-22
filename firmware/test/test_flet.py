import flet as ft
def main(page: ft.Page):
    picker = ft.FilePicker()
    page.overlay.append(picker)
    
    async def click(e):
        print("clicking")
        path = await picker.get_directory_path(dialog_title="Select Dir")
        print("result:", path)
        page.window.close()
        
    btn = ft.ElevatedButton("Test", on_click=click)
    page.add(btn)
    page.update()

ft.run(main)
