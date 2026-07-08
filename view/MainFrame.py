import wx
import os
# import sys
# # 将项目的根目录添加到 sys.path 中
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from model.plot.PointCloudCanvas import PointCloudCanvas

# 颜色常量定义
BG_COLOR_LIGHT_GREEN = wx.Colour(240, 250, 250)  # 浅绿色背景
BG_COLOR_WHITE = wx.WHITE
BG_COLOR_GREEN = wx.Colour(107, 175, 107)  # 绿色（开始按钮）
BG_COLOR_RED = wx.Colour(244, 100, 80)  # 红色（结束按钮）
BG_COLOR_BLUE = wx.Colour(150, 230, 244)  # 钻石蓝（参数按钮）


class BorderedPanel(wx.Panel):
    """带明显黑色边框的面板"""

    def __init__(self, parent, label="", bg_color=None):
        super().__init__(parent)
        self.label = label
        self.bg_color = bg_color or wx.Colour(220, 247, 221)
        self.SetBackgroundColour(self.bg_color)
        self.Bind(wx.EVT_PAINT, self._on_paint)

    def _on_paint(self, event):
        """绘制黑色边框"""
        dc = wx.PaintDC(self)
        rect = self.GetClientRect()

        # 绘制背景
        dc.SetBrush(wx.Brush(self.bg_color))
        dc.DrawRectangle(rect)

        # 绘制粗黑色边框（3像素）
        dc.SetPen(wx.Pen(wx.BLACK, 3))
        dc.SetBrush(wx.TRANSPARENT_BRUSH)
        dc.DrawRectangle(0, 0, rect.width, rect.height)

        # 绘制标签（如果有）
        if self.label:
            font = wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)  # 改为普通字体，不加粗
            dc.SetFont(font)
            dc.SetTextForeground(wx.BLACK)
            text_width, text_height = dc.GetTextExtent(self.label)

            # 标签位置：顶部左侧，内缩一点，向下间隔10像素
            label_x = 15
            label_y = 15

            # 在标签周围地方画白色背景来遮挡边框
            bg_margin = 3  # 标签周围的白色边距
            dc.SetBrush(wx.Brush(self.bg_color))
            dc.SetPen(wx.TRANSPARENT_PEN)
            dc.DrawRectangle(label_x - bg_margin, label_y - bg_margin, text_width + 2 * bg_margin, text_height + 2 * bg_margin)

            # 绘制标签文字
            dc.DrawText(self.label, label_x, label_y)


class ModernButton(wx.Button):
    """现代化按钮类 - 所有按钮显示darker效果和边框"""

    def __init__(self, parent, label, size=wx.Size(150, 50), bg_color=None, text_color=None):
        super().__init__(parent, label=label, size=size)
        self.bg_color = bg_color or BG_COLOR_BLUE  # 原始背景色
        self.text_color = text_color or wx.BLACK  # 文字颜色
        self.disabled_color = wx.Colour(180, 180, 180)  # 禁用时的灰色
        self.visual_active = True
        self.current_text_color = self.text_color

        # 使用普通字体（不加粗）来避免文字重叠
        font = wx.Font(13, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.SetFont(font)

        # 绑定Paint事件
        self.Bind(wx.EVT_PAINT, self._on_paint)

        # 初始化颜色
        self._update_button_appearance()

    def _update_button_appearance(self):
        """根据启用/禁用状态更新按钮外观"""
        if self.IsEnabled() and self.visual_active:
            # 启用时：使用darker效果（减20）
            self.current_bg = wx.Colour(max(0, self.bg_color.red - 20), max(0, self.bg_color.green - 20), max(0, self.bg_color.blue - 20))
            self.current_text_color = self.text_color
        else:
            # 禁用时：使用灰色
            self.current_bg = self.disabled_color
            self.current_text_color = wx.BLACK

        self.SetBackgroundColour(self.current_bg)
        self.SetForegroundColour(self.current_text_color)

    def SetVisualActive(self, active=True):
        """仅控制按钮是否显示彩色外观，不影响点击能力"""
        self.visual_active = active
        self._update_button_appearance()
        self.Refresh()

    def Enable(self, enable=True):
        """重写Enable方法以更新按钮外观"""
        super().Enable(enable)
        self._update_button_appearance()
        self.Refresh()

    def _on_paint(self, event):
        """自定义绘制 - 显示darker效果和边框"""
        dc = wx.PaintDC(self)
        rect = self.GetClientRect()

        # 绘制主背景
        dc.SetBrush(wx.Brush(self.current_bg))
        dc.SetPen(wx.Pen(self.current_bg))
        dc.DrawRectangle(rect)

        # 绘制深色边框（3像素）给按钮凹陷感
        border_color = wx.Colour(max(0, self.current_bg.red - 30), max(0, self.current_bg.green - 30), max(0, self.current_bg.blue - 30))
        dc.SetPen(wx.Pen(border_color, 2))
        dc.SetBrush(wx.TRANSPARENT_BRUSH)
        dc.DrawRectangle(0, 0, rect.width - 1, rect.height - 1)

        # 获取文字
        label = self.GetLabel()
        font = self.GetFont()
        dc.SetFont(font)

        # 测量文字尺寸
        text_width, text_height = dc.GetTextExtent(label)
        x = (rect.width - text_width) // 2
        y = (rect.height - text_height) // 2

        # 绘制文字
        dc.SetTextForeground(self.current_text_color)
        dc.DrawText(label, x, y)


class MainFrame(wx.Frame):
    def __init__(self, parent):
        super().__init__(parent, title="Joihey Software")
        self.SetBackgroundColour(BG_COLOR_LIGHT_GREEN)

        # 尝试加载ico图标（如果存在）
        icon_path = os.path.join(os.path.dirname(__file__), "..", "data", "picture", "junhe.ico")
        if os.path.exists(icon_path):
            icon = wx.Icon(icon_path, wx.BITMAP_TYPE_ICO)
            self.SetIcon(icon)

        self.InitUI()
        # 设置窗口为最大化状态（自适应屏幕尺寸）
        self.Maximize(True)
        self._set_control_buttons_active(False)
        self._set_param_buttons_enabled(False)

    def InitUI(self):
        # 创建滚动窗口
        scrolled_window = wx.ScrolledWindow(self)
        scrolled_window.SetScrollRate(20, 20)
        scrolled_window.SetBackgroundColour(BG_COLOR_LIGHT_GREEN)

        # 主面板
        panel = wx.Panel(scrolled_window)
        panel.SetBackgroundColour(BG_COLOR_LIGHT_GREEN)
        main_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # 左侧操作控制面板
        left_scrolled = wx.ScrolledWindow(panel, size=wx.Size(430, -1))
        left_scrolled.SetScrollRate(10, 10)
        left_scrolled.SetBackgroundColour(BG_COLOR_LIGHT_GREEN)
        left_scrolled.SetMinSize(wx.Size(430, -1))

        left_vbox = wx.BoxSizer(wx.VERTICAL)

        # ===== 操作控制卡片 =====
        control_card = BorderedPanel(left_scrolled, label="☆ 操作控制", bg_color=BG_COLOR_LIGHT_GREEN)
        control_card.SetMinSize(wx.Size(-1, -1))
        control_card_sizer = wx.BoxSizer(wx.VERTICAL)

        # 按钮区域
        btn_sizer1 = wx.BoxSizer(wx.HORIZONTAL)
        self.start_btn = ModernButton(control_card, label="▶ 程序开始运行", size=wx.Size(180, 55), bg_color=BG_COLOR_GREEN, text_color=wx.WHITE)  # 绿色, 白色文字
        self.stop_btn = ModernButton(control_card, label="■ 程序结束运行", size=wx.Size(180, 55), bg_color=BG_COLOR_RED, text_color=wx.WHITE)  # 红色, 白色文字
        btn_sizer1.Add(self.start_btn, 0, wx.ALL, 8)
        btn_sizer1.Add(self.stop_btn, 0, wx.ALL, 8)
        control_card_sizer.AddSpacer(30)
        control_card_sizer.Add(btn_sizer1, 0, wx.EXPAND | wx.TOP | wx.BOTTOM | wx.RIGHT | wx.LEFT, 5)
        control_card.SetSizer(control_card_sizer)

        # ===== 参数设置卡片 =====
        param_card = BorderedPanel(left_scrolled, label="⚙ 参数设置", bg_color=BG_COLOR_LIGHT_GREEN)
        param_card.SetMinSize(wx.Size(-1, -1))
        param_card_sizer = wx.BoxSizer(wx.VERTICAL)

        btn_sizer2 = wx.BoxSizer(wx.HORIZONTAL)
        status_label = wx.StaticText(param_card, label="         左侧 ←                 → 右侧")
        # 加大字体
        status_label_font = wx.Font(11, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        status_label.SetFont(status_label_font)
        status_label.SetForegroundColour(wx.BLACK)
        self.left_out_down_btn = ModernButton(param_card, label="左侧外底(1号)", size=wx.Size(180, 50))
        self.right_in_up_btn = ModernButton(param_card, label="右侧内顶(1号)", size=wx.Size(180, 50))
        btn_sizer2.Add(self.left_out_down_btn, 0, wx.ALL, 8)
        btn_sizer2.Add(self.right_in_up_btn, 0, wx.ALL, 8)

        btn_sizer3 = wx.BoxSizer(wx.HORIZONTAL)
        self.left_out_up_btn = ModernButton(param_card, label="左侧外顶(2号)", size=wx.Size(180, 50))
        self.right_out_up_btn = ModernButton(param_card, label="右侧外顶(2号)", size=wx.Size(180, 50))
        btn_sizer3.Add(self.left_out_up_btn, 0, wx.ALL, 8)
        btn_sizer3.Add(self.right_out_up_btn, 0, wx.ALL, 8)

        btn_sizer4 = wx.BoxSizer(wx.HORIZONTAL)
        self.left_out_lift_btn = ModernButton(param_card, label="左侧二维往复(3号)", size=wx.Size(180, 50))
        self.right_xn_side_btn = ModernButton(param_card, label="右侧侧面云雀(3号)", size=wx.Size(180, 50))
        btn_sizer4.Add(self.left_out_lift_btn, 0, wx.ALL, 8)
        btn_sizer4.Add(self.right_xn_side_btn, 0, wx.ALL, 8)

        param_card_sizer.AddSpacer(50)
        param_card_sizer.Add(status_label, 0, wx.EXPAND | wx.TOP | wx.BOTTOM | wx.RIGHT | wx.LEFT, 5)
        param_card_sizer.AddSpacer(5)
        param_card_sizer.Add(btn_sizer2, 0, wx.EXPAND | wx.TOP | wx.BOTTOM | wx.RIGHT | wx.LEFT, 5)
        param_card_sizer.Add(btn_sizer3, 0, wx.EXPAND | wx.TOP | wx.BOTTOM | wx.RIGHT | wx.LEFT, 5)
        param_card_sizer.Add(btn_sizer4, 0, wx.EXPAND | wx.TOP | wx.BOTTOM | wx.RIGHT | wx.LEFT, 5)
        param_card.SetSizer(param_card_sizer)

        # 添加卡片到左侧布局（不再包括状态输出）
        left_vbox.Add(control_card, 0, wx.EXPAND | wx.ALL, 10)
        left_vbox.Add(param_card, 0, wx.EXPAND | wx.ALL, 10)
        left_vbox.AddStretchSpacer()  # 添加可伸缩的空间

        left_scrolled.SetSizer(left_vbox)

        # 右侧区域 - 包含3D显示和状态输出
        right_panel = wx.Panel(panel)
        right_panel.SetBackgroundColour(BG_COLOR_LIGHT_GREEN)
        right_sizer = wx.BoxSizer(wx.HORIZONTAL)  # 改为水平布局

        # 3D显示区域 - 左侧卡片风格，用BorderedPanel包含
        display_panel = BorderedPanel(right_panel, label="🔷 3D点云画图显示", bg_color=BG_COLOR_LIGHT_GREEN)
        display_panel.SetMinSize(wx.Size(300, 200))
        display_sizer = wx.BoxSizer(wx.VERTICAL)

        self.gl_canvas = PointCloudCanvas(display_panel)
        display_sizer.AddSpacer(30)
        display_sizer.Add(self.gl_canvas, 1, wx.EXPAND | wx.ALL, 8)
        display_panel.SetSizer(display_sizer)

        # 状态输出面板 - 右侧，用BorderedPanel包含
        status_panel = BorderedPanel(right_panel, label="📋 状态输出", bg_color=BG_COLOR_LIGHT_GREEN)
        status_panel.SetMinSize(wx.Size(300, -1))
        status_sizer = wx.BoxSizer(wx.VERTICAL)

        self.status_text = wx.TextCtrl(status_panel, style=wx.TE_MULTILINE | wx.TE_READONLY)
        status_font = wx.Font(9, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.status_text.SetFont(status_font)
        self.status_text.SetBackgroundColour(BG_COLOR_LIGHT_GREEN)  # 浅绿色背景

        status_sizer.AddSpacer(30)
        status_sizer.Add(self.status_text, 1, wx.EXPAND | wx.ALL, 8)
        status_panel.SetSizer(status_sizer)

        # 右侧布局：3D + 状态输出并排
        right_sizer.Add(display_panel, 1, wx.EXPAND | wx.ALL, 10)
        right_sizer.Add(status_panel, 0, wx.EXPAND | wx.TOP | wx.BOTTOM | wx.RIGHT, 10)
        right_panel.SetSizer(right_sizer)

        # 主布局
        main_sizer.Add(left_scrolled, 0, wx.EXPAND | wx.TOP | wx.BOTTOM | wx.LEFT, 10)
        main_sizer.Add(right_panel, 1, wx.EXPAND | wx.TOP | wx.BOTTOM | wx.RIGHT, 10)
        panel.SetSizer(main_sizer)

        # 设置滚动窗口
        scrolled_sizer = wx.BoxSizer(wx.VERTICAL)
        scrolled_sizer.Add(panel, 1, wx.EXPAND)
        scrolled_window.SetSizer(scrolled_sizer)

        # 设置字体
        default_font = wx.SystemSettings.GetFont(wx.SYS_DEFAULT_GUI_FONT)
        self.SetFont(default_font)

        # 设置虚拟大小
        left_scrolled.SetVirtualSize(left_vbox.GetMinSize())

    def _set_param_buttons_enabled(self, enabled: bool):
        """设置参数设置按钮的启用/禁用状态"""
        self.left_out_down_btn.Enable(enabled)
        self.right_in_up_btn.Enable(enabled)
        self.left_out_up_btn.Enable(enabled)
        self.left_out_lift_btn.Enable(enabled)
        self.right_xn_side_btn.Enable(enabled)
        self.right_out_up_btn.Enable(enabled)

    def _set_control_buttons_active(self, active: bool):
        """设置开始/结束按钮是否显示彩色外观"""
        self.start_btn.SetVisualActive(active)
        self.stop_btn.SetVisualActive(active)
        self.start_btn.Enable(True)
        self.stop_btn.Enable(active)

    def on_program_started(self):
        """程序启动后被调用，启用彩色按钮和参数设置按钮"""
        self._set_control_buttons_active(True)
        self._set_param_buttons_enabled(True)

    def on_program_stopped(self):
        """程序停止后被调用，恢复为灰色按钮并禁用参数设置按钮"""
        self._set_control_buttons_active(False)
        self._set_param_buttons_enabled(False)


if __name__ == "__main__":
    app = wx.App()
    frame = MainFrame(None)
    frame.Show()
    app.MainLoop()
