import elsie
from elsie.boxtree.box import Box


def get_static_path(filename: str):
    static_dir = "../static/"
    return static_dir + filename


def get_image_path(filename: str):
    return get_static_path("images/" + filename)


def split_lines(input: str, lines: int):
    input = input.split("\n")
    before = "\n".join(input[:lines])
    after = "\n".join(input[lines:])
    return (before, after)


def n_empty_trailing(input: str):
    input = input.split("\n")
    input.reverse()
    for i, e in enumerate(input[1:]):
        if e != "":
            return i
    return 0


def init_deck():
    slides = elsie.SlideDeck(width=1920, height=1080)
    slides.update_style("default", elsie.TextStyle(font="Lato", align="left", size=64))
    slides.update_style("code", elsie.TextStyle(size=38))
    slides.set_style("link", elsie.TextStyle(color="blue"))
    grayed = slides.get_style("code")
    grayed.color = "gray"
    slides.set_style("grayed", grayed)
    return slides


def page_numbering(slides: elsie.SlideDeck):
    for i, slide in enumerate(slides):
        slide.box(x="[100%]", y="[100%]", p_right=30, p_bottom=30).text(
            f"{i}", elsie.TextStyle(align="right")
        )


def render_deck(slides: elsie.SlideDeck, filename: str):
    slides.render(
        get_static_path("pdf/" + filename), slide_postprocessing=page_numbering
    )


def logo_header_slide(parent: Box, title: str):
    parent.box(x=1570, y=40).image(get_image_path("picknik_logo.png"))
    parent.sbox(name="header", x=0, height=140).fbox(p_left=20).text(
        title, elsie.TextStyle(bold=True)
    )
    return parent.fbox(name="content", p_left=20, p_right=20)


def image_slide(parent: Box, title: str, image_path: str):
    content = logo_header_slide(parent, title)
    content = content.fbox(horizontal=True, p_top=20, p_bottom=20)
    text_area = content.fbox(name="text_area", width="50%")
    content.sbox(name="image", width="fill").image(image_path)
    return text_area


def section_title_slide(parent: Box, title: str, subtitle: str):
    content = logo_header_slide(parent, "")
    content.sbox().text(title, elsie.TextStyle(align="right", size=240, bold=True))
    content.box().text(subtitle)


# Returns the code object so you can do line highlighting
def code_slide(parent: Box, title: str, language: str, code: str):
    content = logo_header_slide(parent, title)
    code_bg = "#EAEAEA"
    box = content.box(y=0, width="100%", height="100%", p_bottom=20, z_level=-2)
    box.rect(bg_color=code_bg, rx=20, ry=20)
    return (
        box.overlay()
        .sbox(x=0, y=0, p_left=20, p_right=20, p_top=20, p_bottom=20, z_level=0)
        .code(language, code)
    )


def grayed_before_after_code_slide(
    parent: Box,
    title: str,
    language: str,
    code: str,
    code_start: int,
    code_lines: int,
):
    content = logo_header_slide(parent, title)
    code_bg = "#EAEAEA"
    box = content.box(y=0, width="100%", height="100%", p_bottom=20, z_level=-2)
    box.rect(bg_color=code_bg, rx=20, ry=20)
    overlay = box.overlay()
    (before, code) = split_lines(code, code_start)
    (code, after) = split_lines(code, code_lines)
    before_box = overlay.sbox(
        name="before", x=0, y=0, p_left=20, p_right=20, p_top=20, p_bottom=20, z_level=0
    )
    before_box.code("", before, style="grayed")
    code_box = overlay.sbox(
        name="code", x=0, y=before_box.y("100%"), z_level=0, p_left=20, p_right=20
    )
    code_box.code(language, code)
    after_box = overlay.sbox(
        name="after", x=0, y=code_box.y("100%"), z_level=0, p_left=20, p_right=20
    )
    after_box.code("", after, style="grayed")
