from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import Screen, ScreenManager
from kivy.properties import StringProperty, NumericProperty, ObjectProperty
from crawl import *



class LottoScreen(Screen):
    one = NumericProperty(select_index[0][0])
    two = NumericProperty(select_index[1][0])
    three = NumericProperty(select_index[2][0])
    four = NumericProperty(select_index[3][0])
    five = NumericProperty(select_index[4][0])
    six = NumericProperty(select_index[5][0])
    seven = NumericProperty(select_index[6][0])
    this_win = ObjectProperty(this_winninnum)
    this_prize = ObjectProperty(this_winamnt)
    this_round = NumericProperty(endRound)
    pass
class HomeScreen(Screen):
    pass
class SettingScreen(Screen):
    pass

x = 1

GUI = Builder.load_file("main.kv")
class MainApp(App):

    def build(self):

        return GUI

    def change_screen(self, screen_name):
        # asd
        screen_manger = self.root.ids['screen_manager']
        print(screen_name)
        screen_manger.current = screen_name
        #screen_manager = self.root.ids

MainApp().run()