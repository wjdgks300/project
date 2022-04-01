from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import Screen, ScreenManager
from kivy.properties import StringProperty

class LottoScreen(Screen):
    pass
class HomeScreen(Screen):
    pass
class SettingScreen(Screen):
    pass

x = 1

GUI = Builder.load_file("main.kv")
class MainApp(App):
    a = StringProperty("test")
    def build(self):
        self.a = StringProperty("test")
        return GUI

    def change_screen(self, screen_name):
        # asd
        screen_manger = self.root.ids['screen_manager']
        print(screen_name)
        screen_manger.current = screen_name
        #screen_manager = self.root.ids

MainApp().run()