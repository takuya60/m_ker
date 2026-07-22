import flet as ft
import asyncio

async def main(page: ft.Page):
    picker = ft.FilePicker()
    try:
        path = await picker.get_directory_path()
        print("Success without overlay")
    except Exception as e:
        print("Error without overlay:", repr(e))
        try:
            page.overlay.append(picker)
            page.update()
            print("Added to overlay")
        except Exception as e2:
            print("Error adding to overlay:", repr(e2))
    page.window.close()

ft.app(main)
