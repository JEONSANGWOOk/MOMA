"""Shared industrial desktop palette and compact widget styles."""
BG = '#e9edf2'
PANEL = '#f5f7fa'
WHITE = '#ffffff'
INK = '#263445'
MUTED = '#65758a'
CHROME = '#243449'
BLUE = '#2376cf'
GREEN = '#159e75'
RED = '#c93043'
ORANGE = '#b77915'
DARK = '#ffffff'
GRID = '#e6ebf1'


def configure_styles(style, font):
    style.theme_use('clam')
    style.configure('.', font=(font, 9), background=PANEL, foreground=INK)
    style.configure('TFrame', background=BG)
    style.configure('TLabel', background=BG, foreground=INK)
    style.configure('TButton', padding=(8, 4), background=PANEL, foreground=INK)
    style.map('TButton', background=[('active', '#dce9f7')])
    style.configure('TEntry', fieldbackground=WHITE, foreground=INK, padding=4)
    style.configure('TCombobox', fieldbackground=WHITE, foreground=INK, padding=3)
    style.map('TCombobox', fieldbackground=[('readonly', WHITE)], foreground=[('readonly', INK)])
    style.configure('Treeview', rowheight=25, background=WHITE, fieldbackground=WHITE, foreground=INK, borderwidth=1)
    style.configure('Treeview.Heading', padding=5, background='#e4eaf1', foreground=INK, font=(font, 9, 'bold'))
    style.map('Treeview', background=[('selected', BLUE)], foreground=[('selected', WHITE)])
    style.configure('TNotebook', background=BG, borderwidth=0)
    style.configure('TNotebook.Tab', padding=(12, 6), background='#e0e6ed', foreground=MUTED)
    style.map('TNotebook.Tab', background=[('selected', WHITE)], foreground=[('selected', BLUE)])
    style.layout('Workspace.TNotebook.Tab', [])
    style.configure('Workspace.TNotebook', background=BG, borderwidth=0)
    style.configure('TPanedwindow', background=BG)
