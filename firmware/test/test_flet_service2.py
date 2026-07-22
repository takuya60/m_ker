import flet as ft
async def main(page: ft.Page):
    picker = ft.FilePicker()
    try:
        await picker.get_directory_path()
        print('SUCCESS')
    except Exception as e:
        print('ERROR:', repr(e))
    page.window.close()
ft.run(main)
