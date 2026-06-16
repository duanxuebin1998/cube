# -*- coding: utf-8 -*-
"""Build a static documentation site under D:\\CUBE\\site.

The site keeps the original docs relative layout so existing CPU2/CPU3 cross
links continue to work, while adding a root homepage, search index, library page
and deployment notes.
"""

from __future__ import annotations

import datetime as dt
import html
import json
import os
import re
import shutil
from html.parser import HTMLParser
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple
from urllib.parse import unquote, urlparse


ROOT = Path(__file__).resolve().parents[1]
SITE = ROOT / "site"

SOURCE_ROOTS = [
    ROOT / "docs",
    ROOT / "LTD_MAIN_CPU2" / "docs",
    ROOT / "LTD_DISPLAY_CPU3" / "docs",
]

INITIAL_EXTS = {
    ".html",
    ".htm",
    ".md",
    ".css",
    ".js",
    ".json",
    ".png",
    ".jpg",
    ".jpeg",
    ".gif",
    ".svg",
    ".webp",
    ".ico",
}

REFERENCED_EXTS = INITIAL_EXTS | {
    ".pdf",
    ".docx",
    ".xlsx",
    ".txt",
    ".dat",
}

SOURCE_CODE_EXTS = {
    ".c",
    ".h",
    ".cpp",
    ".hpp",
    ".s",
    ".ld",
    ".ioc",
}

SKIP_SCHEMES = ("http:", "https:", "mailto:", "javascript:", "data:")
SITE_BUILD_DATE = dt.datetime.now().strftime("%Y-%m-%d %H:%M")
VENDOR_RAW_HTML_PREFIXES = (
    "docs/06_SIL功能安全认证/00_外部资料/03_STM32与X-CUBE-STL/02_X-CUBE-STL-F4_V2.0.0包内文档/",
)


KEY_LINKS = [
    {
        "title": "程序流程统一入口",
        "desc": "从整机业务链路进入 CPU2/CPU3 详细流程图。",
        "href": "docs/00_程序流程导航/index.html",
        "tag": "流程",
    },
    {
        "title": "跨 CPU 业务链路",
        "desc": "液位、水位、密度、参数、故障和外部协议的跨 CPU 路线。",
        "href": "docs/00_程序流程导航/跨CPU业务链路.html",
        "tag": "链路",
    },
    {
        "title": "CPU2 程序流程总览",
        "desc": "CPU2 测量、运动、故障、通信、参数和外设流程入口。",
        "href": "LTD_MAIN_CPU2/docs/00_程序流程/CPU2程序流程总览.html",
        "tag": "CPU2",
    },
    {
        "title": "CPU3 程序流程总览",
        "desc": "CPU3 显示、菜单、外部协议、CPU2 轮询和本机参数流程入口。",
        "href": "LTD_DISPLAY_CPU3/docs/00_程序流程/CPU3程序流程总览.html",
        "tag": "CPU3",
    },
    {
        "title": "CPU2 问题点总表",
        "desc": "CPU2 程序流程梳理后的风险、问题点和整改建议汇总。",
        "href": "LTD_MAIN_CPU2/docs/00_程序流程/问题点总表.html",
        "tag": "问题",
    },
    {
        "title": "LNG 计量仪系统框图",
        "desc": "整机拓扑、硬件连接、数据闭环和外部协议总览。",
        "href": "docs/LNG计量仪系统框图.html",
        "tag": "设备",
    },
    {
        "title": "LNG 计量仪设备说明书",
        "desc": "设备结构、测量流程、通信接口、参数维护和故障处理说明。",
        "href": "docs/LNG计量仪设备说明书.html",
        "tag": "说明书",
    },
]


CATEGORY_RULES = [
    ("docs/00_程序流程导航", "程序流程导航", "流程"),
    ("LTD_MAIN_CPU2/docs/00_程序流程", "CPU2 程序流程", "CPU2"),
    ("LTD_DISPLAY_CPU3/docs/00_程序流程", "CPU3 程序流程", "CPU3"),
    ("LTD_MAIN_CPU2/docs/01_电机与编码器", "CPU2 电机与编码器", "CPU2"),
    ("LTD_MAIN_CPU2/docs/02_通信与解耦", "CPU2 通信与解耦", "CPU2"),
    ("docs/00_构建与版本", "构建与版本", "版本"),
    ("docs/01_协议与寄存器", "协议与寄存器", "协议"),
    ("docs/02_需求与计划", "需求与计划", "需求"),
    ("docs/03_问题分析与整改", "问题分析与整改", "问题"),
    ("docs/04_界面与菜单", "界面与菜单", "界面"),
    ("docs/05_测试记录", "测试记录", "测试"),
    ("docs/06_SIL功能安全认证", "SIL 功能安全认证", "SIL"),
    ("docs/07_硬件原理图", "硬件原理图", "硬件"),
    ("docs", "工程文档", "文档"),
]


class TextExtractor(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.skip_depth = 0
        self.parts: List[str] = []
        self.title_parts: List[str] = []
        self.heading_parts: List[str] = []
        self.current_tag: Optional[str] = None

    def handle_starttag(self, tag: str, attrs: Sequence[Tuple[str, Optional[str]]]) -> None:
        if tag in {"script", "style"}:
            self.skip_depth += 1
        self.current_tag = tag

    def handle_endtag(self, tag: str) -> None:
        if tag in {"script", "style"} and self.skip_depth > 0:
            self.skip_depth -= 1
        self.current_tag = None

    def handle_data(self, data: str) -> None:
        if self.skip_depth:
            return
        text = " ".join(data.split())
        if not text:
            return
        self.parts.append(text)
        if self.current_tag == "title":
            self.title_parts.append(text)
        elif self.current_tag in {"h1", "h2", "h3"}:
            self.heading_parts.append(text)


def clean_text(text: str, max_len: int = 320) -> str:
    normalized = " ".join(text.split())
    return normalized[:max_len].rstrip()


def is_inside(path: Path, parent: Path) -> bool:
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


def ensure_site_reset() -> None:
    site_resolved = SITE.resolve()
    root_resolved = ROOT.resolve()
    if site_resolved == root_resolved or not is_inside(site_resolved, root_resolved):
        raise RuntimeError(f"Refuse to reset unsafe site path: {site_resolved}")
    if SITE.exists():
        shutil.rmtree(SITE)
    SITE.mkdir(parents=True, exist_ok=True)


def copy_file(src: Path, copied: Set[Path]) -> None:
    src = src.resolve()
    if src in copied or not src.is_file():
        return
    if not is_inside(src, ROOT):
        return
    rel = src.relative_to(ROOT)
    dst = SITE / rel
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    copied.add(src)


def iter_source_files() -> Iterable[Path]:
    for source_root in SOURCE_ROOTS:
        if not source_root.exists():
            continue
        for path in source_root.rglob("*"):
            if path.is_file():
                yield path


def copy_initial_files(copied: Set[Path]) -> None:
    for path in iter_source_files():
        if path.suffix.lower() in INITIAL_EXTS:
            copy_file(path, copied)


def extract_refs(path: Path, text: str) -> List[str]:
    refs = []
    if path.suffix.lower() in {".html", ".htm"}:
        refs.extend(m.group(2) for m in re.finditer(r"\b(href|src)\s*=\s*['\"]([^'\"]+)['\"]", text, flags=re.I))
    elif path.suffix.lower() == ".md":
        refs.extend(m.group(1) for m in re.finditer(r"\[[^\]]+\]\(([^)]+)\)", text))
    if path.suffix.lower() in {".css", ".html", ".htm"}:
        refs.extend(m.group(1).strip("'\"") for m in re.finditer(r"url\(([^)]+)\)", text, flags=re.I))
    return refs


def ref_to_source_path(base_file: Path, ref: str) -> Optional[Path]:
    ref = ref.strip()
    if not ref or ref.startswith("#") or ref.startswith("//") or ref.lower().startswith(SKIP_SCHEMES):
        return None
    parsed = urlparse(ref)
    if parsed.scheme == "file":
        target = Path(unquote(parsed.path.lstrip("/")))
        if re.match(r"^[A-Za-z]:", str(target)):
            return target
        return None
    if parsed.scheme:
        return None
    raw_path = ref.split("#", 1)[0].split("?", 1)[0]
    if not raw_path:
        return None
    raw_path = unquote(raw_path).replace("\\", "/")
    if raw_path.startswith("/"):
        candidate = ROOT / raw_path.lstrip("/")
    else:
        candidate = base_file.parent / raw_path
    try:
        candidate = candidate.resolve()
    except OSError:
        return None
    if not is_inside(candidate, ROOT):
        return None
    return candidate


def copy_referenced_files(copied: Set[Path]) -> None:
    for _ in range(4):
        before = len(copied)
        for src in list(copied):
            if src.suffix.lower() not in {".html", ".htm", ".md", ".css"}:
                continue
            try:
                text = src.read_text(encoding="utf-8", errors="ignore")
            except OSError:
                continue
            for ref in extract_refs(src, text):
                target = ref_to_source_path(src, ref)
                if target is not None and target.exists() and target.is_file():
                    if target.suffix.lower() in REFERENCED_EXTS:
                        copy_file(target, copied)
        if len(copied) == before:
            break


def rel_url(path: Path) -> str:
    return path.relative_to(SITE).as_posix()


def category_for(rel: str) -> Tuple[str, str]:
    for prefix, category, tag in CATEGORY_RULES:
        if rel.startswith(prefix):
            return category, tag
    return "文档库", "文档"


def priority_for(rel: str, title: str) -> int:
    priority = 10
    for index, item in enumerate(KEY_LINKS):
        if rel == item["href"]:
            return 100 - index
    if "CPU2程序流程总览" in rel or "CPU3程序流程总览" in rel:
        priority = 90
    elif "00_程序流程" in rel:
        priority = 75
    elif "问题" in rel or "整改" in rel:
        priority = 65
    elif "协议" in rel or "寄存器" in rel:
        priority = 60
    elif "版本" in rel or "CHANGELOG" in rel:
        priority = 55
    elif title.endswith("README"):
        priority = 35
    return priority


def extract_html_record(path: Path) -> Dict[str, object]:
    text = path.read_text(encoding="utf-8", errors="ignore")
    parser = TextExtractor()
    parser.feed(text)
    rel = rel_url(path)
    title = clean_text(" ".join(parser.title_parts)) or clean_text(" ".join(parser.heading_parts[:1])) or path.stem
    heading_text = clean_text(" ".join(parser.heading_parts[:6]), 500)
    body_text = clean_text(" ".join(parser.parts), 1400)
    summary = heading_text or clean_text(body_text, 220)
    category, tag = category_for(rel)
    keywords = sorted(set(re.findall(r"[A-Za-z0-9_./+-]+|[\u4e00-\u9fff]{2,}", title + " " + summary + " " + rel)))[:80]
    return {
        "id": re.sub(r"[^A-Za-z0-9_]+", "-", rel).strip("-").lower(),
        "title": title,
        "category": category,
        "summary": summary,
        "url": rel,
        "keywords": keywords,
        "type": "html",
        "scope": tag,
        "module": "CPU2+CPU3" if "CPU2" in body_text and "CPU3" in body_text else ("CPU2" if "CPU2" in body_text or "LTD_MAIN_CPU2" in rel else ("CPU3" if "CPU3" in body_text or "LTD_DISPLAY_CPU3" in rel else "")),
        "tags": [tag, category],
        "updated": dt.datetime.fromtimestamp(path.stat().st_mtime).strftime("%Y-%m-%d"),
        "sourcePath": rel,
        "priority": priority_for(rel, title),
        "text": body_text,
    }


def extract_markdown_record(path: Path) -> Dict[str, object]:
    text = path.read_text(encoding="utf-8", errors="ignore")
    rel = rel_url(path)
    headings = [line.strip("# ").strip() for line in text.splitlines() if line.startswith("#")]
    title = clean_text(headings[0]) if headings else path.stem
    body = clean_text(re.sub(r"[#`>*_\-\[\]()]+" , " ", text), 1200)
    category, tag = category_for(rel)
    return {
        "id": re.sub(r"[^A-Za-z0-9_]+", "-", rel).strip("-").lower(),
        "title": title,
        "category": category,
        "summary": clean_text(body, 260),
        "url": rel,
        "keywords": sorted(set(re.findall(r"[A-Za-z0-9_./+-]+|[\u4e00-\u9fff]{2,}", title + " " + body + " " + rel)))[:80],
        "type": "md",
        "scope": tag,
        "module": "CPU2+CPU3" if "CPU2" in body and "CPU3" in body else ("CPU2" if "CPU2" in body or "LTD_MAIN_CPU2" in rel else ("CPU3" if "CPU3" in body or "LTD_DISPLAY_CPU3" in rel else "")),
        "tags": [tag, category],
        "updated": dt.datetime.fromtimestamp(path.stat().st_mtime).strftime("%Y-%m-%d"),
        "sourcePath": rel,
        "priority": priority_for(rel, title),
        "text": body,
    }


def build_search_index() -> List[Dict[str, object]]:
    records: List[Dict[str, object]] = []
    for path in SITE.rglob("*"):
        if not path.is_file():
            continue
        if path.name == "search-index.json":
            continue
        if path.suffix.lower() in {".html", ".htm"}:
            records.append(extract_html_record(path))
        elif path.suffix.lower() == ".md":
            records.append(extract_markdown_record(path))
    records.sort(key=lambda item: (-int(item["priority"]), str(item["category"]), str(item["title"])))
    return records


def write_site_file(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8", newline="\n")


def shell_link(href: str, label: str, klass: str = "pill") -> str:
    return f'<a class="{klass}" href="{html.escape(href)}">{html.escape(label)}</a>'


def build_home(records: Sequence[Dict[str, object]]) -> str:
    counts: Dict[str, int] = {}
    for record in records:
        counts[str(record["category"])] = counts.get(str(record["category"]), 0) + 1
    category_cards = []
    for category, count in sorted(counts.items(), key=lambda item: (-item[1], item[0])):
        category_cards.append(
            f'<a class="category-card" href="library.html?cat={html.escape(category)}">'
            f"<strong>{html.escape(category)}</strong><span>{count} 个条目</span></a>"
        )
    quick_cards = []
    for item in KEY_LINKS:
        quick_cards.append(
            '<article class="quick-card">'
            f'<span class="tag">{html.escape(item["tag"])}</span>'
            f'<h3>{html.escape(item["title"])}</h3>'
            f'<p>{html.escape(item["desc"])}</p>'
            f'<a href="{html.escape(item["href"])}">打开</a>'
            "</article>"
        )
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CUBE 文档中心</title>
<link rel="stylesheet" href="assets/site.css">
</head>
<body>
<header class="site-topbar">
<a class="brand" href="index.html">CUBE 文档中心</a>
<nav>
<a href="flow.html">程序流程</a>
<a href="library.html">文档库</a>
<a href="deploy.html">发布说明</a>
</nav>
</header>
<main class="site-wrap">
<section class="home-hero">
<div>
<p class="eyebrow">工程文档总入口</p>
<h1>CPU2、CPU3、协议、问题和流程图统一检索</h1>
<p>保留原有文档目录骨架，新增站点首页、分类导航和站内搜索。建议用 HTTP 服务访问，不以 file:// 作为验收口径。</p>
<div class="hero-actions">
{shell_link("docs/00_程序流程导航/index.html", "程序流程统一入口")}
{shell_link("docs/00_程序流程导航/跨CPU业务链路.html", "跨 CPU 业务链路")}
{shell_link("library.html", "打开文档库")}
</div>
</div>
<aside class="status-panel">
<h2>站点状态</h2>
<dl>
<div><dt>生成时间</dt><dd>{SITE_BUILD_DATE}</dd></div>
<div><dt>索引条目</dt><dd>{len(records)}</dd></div>
<div><dt>HTML 页面</dt><dd>{sum(1 for r in records if r["type"] == "html")}</dd></div>
<div><dt>Markdown 页面</dt><dd>{sum(1 for r in records if r["type"] == "md")}</dd></div>
</dl>
</aside>
</section>
<section class="search-panel">
<div class="section-head"><h2>全站搜索</h2><span>支持中文、函数名、命令、协议、路径片段</span></div>
<div class="search-row">
<input class="site-search" data-search-input placeholder="输入关键词，例如 CPU2、液位、SI7000、AD5421、FollowOilLevel">
<button data-search-clear>清空</button>
</div>
<div class="filter-row" data-filter-row>
<button data-filter="all" class="is-active">全部</button>
<button data-filter="流程">流程</button>
<button data-filter="CPU2">CPU2</button>
<button data-filter="CPU3">CPU3</button>
<button data-filter="协议">协议</button>
<button data-filter="问题">问题</button>
<button data-filter="版本">版本</button>
<button data-filter="SIL">SIL</button>
</div>
<div class="search-results" data-search-results></div>
</section>
<section>
<div class="section-head"><h2>常用入口</h2><span>按工程任务组织，不按文件夹机械排列</span></div>
<div class="quick-grid">
{''.join(quick_cards)}
</div>
</section>
<section>
<div class="section-head"><h2>分类导航</h2><span>进入对应业务域继续查找证据和流程</span></div>
<div class="category-grid">
{''.join(category_cards)}
</div>
</section>
</main>
<script src="assets/site.js"></script>
</body>
</html>
"""


def build_flow_page() -> str:
    def flow_links(root_rel: str, preferred: Sequence[str]) -> str:
        root = ROOT / root_rel
        seen: Set[str] = set()
        names: List[str] = []
        for name in preferred:
            if (root / name).is_file():
                names.append(name)
                seen.add(name)
        for path in sorted(root.glob("*.html"), key=lambda item: item.name):
            if path.name not in seen:
                names.append(path.name)
                seen.add(path.name)
        return "".join(shell_link(f"{root_rel}/{name}", Path(name).stem) for name in names)

    cpu2 = flow_links(
        "LTD_MAIN_CPU2/docs/00_程序流程",
        ["CPU2程序流程总览.html", "问题点总表.html"],
    )
    cpu3 = flow_links(
        "LTD_DISPLAY_CPU3/docs/00_程序流程",
        ["CPU3程序流程总览.html"],
    )
    return f"""<!doctype html>
<html lang="zh-CN">
<head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>程序流程导航</title><link rel="stylesheet" href="assets/site.css"></head>
<body>
<header class="site-topbar"><a class="brand" href="index.html">CUBE 文档中心</a><nav><a href="flow.html">程序流程</a><a href="library.html">文档库</a><a href="deploy.html">发布说明</a></nav></header>
<main class="site-wrap">
<section class="page-title"><p class="eyebrow">Program Flow</p><h1>程序流程导航</h1><p>这里集中进入 CPU2、CPU3 和跨 CPU 业务链路。详细流程仍在各自 HTML 中维护。</p></section>
<section class="split-grid">
<article class="panel"><h2>整机链路</h2><p>先从统一入口确认整体阅读顺序，再按跨 CPU 链路进入详细页面。</p><div class="link-cloud">{shell_link("docs/00_程序流程导航/index.html", "程序流程统一入口")}{shell_link("docs/00_程序流程导航/跨CPU业务链路.html", "跨 CPU 业务链路")}</div></article>
<article class="panel"><h2>重点问题</h2><p>流程图中整理的问题点集中在 CPU2 问题点总表和问题分析目录。</p><div class="link-cloud">{shell_link("LTD_MAIN_CPU2/docs/00_程序流程/问题点总表.html", "CPU2 问题点总表")}{shell_link("library.html?cat=问题分析与整改", "问题分析与整改")}</div></article>
</section>
<section><div class="section-head"><h2>CPU2 程序流程</h2><span>测量执行、运动控制、通信发布和故障管理</span></div><div class="link-cloud">{cpu2}</div></section>
<section><div class="section-head"><h2>CPU3 程序流程</h2><span>显示菜单、外部协议、CPU2 轮询和本机参数</span></div><div class="link-cloud">{cpu3}</div></section>
</main>
</body></html>
"""


def build_library_page() -> str:
    return """<!doctype html>
<html lang="zh-CN">
<head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>文档库</title><link rel="stylesheet" href="assets/site.css"></head>
<body>
<header class="site-topbar"><a class="brand" href="index.html">CUBE 文档中心</a><nav><a href="flow.html">程序流程</a><a href="library.html">文档库</a><a href="deploy.html">发布说明</a></nav></header>
<main class="site-wrap">
<section class="page-title"><p class="eyebrow">Library</p><h1>文档库</h1><p>按分类、关键词和路径查找 HTML 与 Markdown 文档。</p></section>
<section class="search-panel">
<div class="search-row"><input class="site-search" data-search-input placeholder="搜索标题、路径、协议、函数、问题关键词"><button data-search-clear>清空</button></div>
<div class="filter-row" data-filter-row><button data-filter="all" class="is-active">全部</button><button data-filter="流程">流程</button><button data-filter="CPU2">CPU2</button><button data-filter="CPU3">CPU3</button><button data-filter="协议">协议</button><button data-filter="问题">问题</button><button data-filter="需求">需求</button><button data-filter="SIL">SIL</button><button data-filter="硬件">硬件</button></div>
</section>
<noscript><section class="panel"><h2>无脚本备用入口</h2><p>当前浏览器未启用脚本，文档库筛选不可用，可先进入程序流程或首页重点入口。</p><div class="link-cloud"><a href="flow.html">程序流程</a><a href="docs/00_程序流程导航/index.html">程序流程统一入口</a><a href="docs/00_程序流程导航/跨CPU业务链路.html">跨 CPU 业务链路</a></div></section></noscript>
<section><div class="section-head"><h2>文档列表</h2><span data-doc-count></span></div><div class="doc-list" data-doc-list></div></section>
</main>
<script src="assets/site.js"></script>
</body></html>
"""


def build_deploy_page() -> str:
    return """<!doctype html>
<html lang="zh-CN">
<head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>静态网站发布说明</title><link rel="stylesheet" href="assets/site.css"></head>
<body>
<header class="site-topbar"><a class="brand" href="index.html">CUBE 文档中心</a><nav><a href="flow.html">程序流程</a><a href="library.html">文档库</a><a href="deploy.html">发布说明</a></nav></header>
<main class="site-wrap">
<section class="page-title"><p class="eyebrow">Deploy</p><h1>静态网站发布说明</h1><p>本网站是静态文件目录，不依赖前端框架。最终验收请使用 HTTP 访问，不建议用 file://。</p></section>
<section><h2>本地预览</h2><pre><code>cd /d D:\\CUBE
py -m http.server 8128 -d D:\\CUBE\\site</code></pre><p>打开 <code>http://127.0.0.1:8128/</code>。</p></section>
<section><h2>内网临时共享</h2><pre><code>cd /d D:\\CUBE
py -m http.server 8128 --bind 0.0.0.0 -d D:\\CUBE\\site</code></pre><p>同网段访问 <code>http://&lt;本机IP&gt;:8128/</code>。长期发布建议使用 IIS 或 Nginx。</p></section>
<section><h2>Nginx 示例</h2><pre><code>server {
    listen 8080;
    server_name _;
    charset utf-8;
    root D:/CUBE/site;
    index index.html;
    location / {
        try_files $uri $uri/ =404;
    }
}</code></pre></section>
<section><h2>发布前验证</h2><ul class="check-list"><li>运行 <code>py tools\\build_static_docs_site.py</code> 重新生成站点。</li><li>确认脚本输出 CUBE 自有 HTML 断链数为 0。</li><li>用 HTTP 打开首页、程序流程、跨 CPU 链路、CPU2 总览、CPU3 总览。</li><li>检查搜索框、中文路径、CSS/JS 加载、无整页横向溢出。</li><li>ST 官方包内 Release Notes 保留原始相对链接，其中部分指向未归档的 Drivers/Middlewares/Projects 解压源码树，按供应商原始资料附件处理，不作为 CUBE 自有文档断链。</li><li>不要只用 file:// 验收，因为相对路径和 JS 行为与 HTTP 不完全一致。</li></ul></section>
</main>
</body></html>
"""


def build_css() -> str:
    return """
:root{--ink:#17233a;--muted:#5c6d82;--line:#d8e3ef;--bg:#f3f7fb;--panel:#fff;--blue:#2f73bd;--green:#208765;--amber:#b87616;--red:#bb4f59;--shadow:0 12px 30px rgba(31,59,91,.09)}
*{box-sizing:border-box}
html{overflow-x:hidden}
body{margin:0;background:linear-gradient(180deg,#f6f9fd 0,#eef4fa 100%);color:var(--ink);font:15px/1.72 "Microsoft YaHei",Segoe UI,Arial,sans-serif;overflow-x:hidden}
a{color:#1f65aa;text-decoration:none}
code{font-family:Consolas,"Courier New",monospace;background:#eef4fb;border:1px solid #d7e4f2;border-radius:5px;padding:1px 5px;overflow-wrap:anywhere;word-break:break-all}
pre{overflow:auto;background:#122239;color:#eef7ff;border-radius:8px;padding:14px}
table{display:block;max-width:100%;overflow-x:auto;border-collapse:collapse}
th,td{word-break:break-word}
.site-topbar{position:sticky;top:0;z-index:10;display:flex;justify-content:space-between;gap:18px;align-items:center;padding:10px 28px;background:rgba(255,255,255,.94);border-bottom:1px solid var(--line);backdrop-filter:blur(8px)}
.brand{font-weight:900;color:#16385e}
.site-topbar nav{display:flex;flex-wrap:wrap;gap:8px}
.site-topbar nav a{padding:6px 9px;border-radius:6px;color:#284c70;font-weight:700}
.site-wrap{max-width:1240px;margin:0 auto;padding:26px 28px 64px}
.home-hero,.page-title,section{background:var(--panel);border:1px solid var(--line);border-radius:10px;box-shadow:0 8px 24px rgba(31,59,91,.06)}
.home-hero{display:grid;grid-template-columns:minmax(0,1.5fr) 340px;gap:18px;padding:28px}
.page-title,section{padding:22px 24px;margin:18px 0}
.eyebrow{margin:0 0 6px;color:#57708b;font-size:12px;font-weight:900;text-transform:uppercase;letter-spacing:0}
h1{margin:0 0 10px;font-size:30px;line-height:1.25}
h2{margin:0 0 10px;font-size:22px}
h3{margin:0 0 8px;font-size:17px}
p{color:#42556c;margin:0 0 12px}
.hero-actions,.link-cloud,.filter-row,.quick-card .tag,.links{display:flex;flex-wrap:wrap;gap:8px}
.pill,.quick-card a,.filter-row button,.search-row button{display:inline-flex;align-items:center;min-height:30px;padding:5px 10px;border:1px solid #d6e4f3;border-radius:999px;background:#f5f9fe;color:#255d96;font-weight:800;font-size:13px}
.status-panel{border:1px solid #cfe0f2;border-radius:9px;background:#f8fbff;padding:16px}
.status-panel dl{margin:0;display:grid;gap:8px}
.status-panel div{display:flex;justify-content:space-between;gap:12px;border-bottom:1px solid #e2ebf5;padding-bottom:6px}
.status-panel dt{color:#62758a}.status-panel dd{margin:0;font-weight:900}
.section-head{display:flex;justify-content:space-between;gap:12px;align-items:baseline;margin-bottom:12px}
.section-head span{color:#66798f;font-size:13px}
.search-panel{background:#fbfdff}
.search-row{display:grid;grid-template-columns:1fr auto;gap:10px;margin-bottom:10px}
.site-search{width:100%;border:1px solid #cfddeb;border-radius:8px;padding:11px 12px;font:inherit;color:var(--ink);background:#fff}
.filter-row button{border-radius:6px;cursor:pointer}
.filter-row button.is-active{background:#2f73bd;color:#fff;border-color:#2f73bd}
.quick-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(255px,1fr));gap:12px}
.quick-card,.result-card,.doc-card,.panel{border:1px solid #dbe6f2;border-radius:8px;background:#fbfdff;padding:14px}
.quick-card .tag{display:inline-flex;color:#49647f;background:#eef5fc;margin-bottom:8px}
.quick-card p,.result-card p,.doc-card p{font-size:13px}
.category-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:10px}
.category-card{display:flex;justify-content:space-between;gap:10px;border:1px solid #dbe6f2;border-radius:8px;background:#fbfdff;padding:12px;color:var(--ink)}
.category-card span{color:#63778d;font-size:13px}
.search-results,.doc-list{display:grid;gap:10px;margin-top:12px}
.result-card strong,.doc-card strong{display:block;color:#16263b}
.result-meta{display:flex;flex-wrap:wrap;gap:8px;color:#677a90;font-size:12px;margin-top:8px}
.split-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px}
.check-list{margin:0;padding-left:20px;color:#42556c}
@media(max-width:820px){.site-topbar{display:block;padding:10px 16px}.site-topbar nav{margin-top:8px}.site-wrap{padding:16px}.home-hero{grid-template-columns:1fr;padding:20px}.search-row{grid-template-columns:1fr}.section-head{display:block}h1{font-size:24px}}
""".strip()


def build_js() -> str:
    return """
(function(){
  const state = { docs: [], filter: 'all', query: '' };
  const norm = (s) => String(s || '').toLowerCase();
  const params = new URLSearchParams(location.search);
  function score(item, q) {
    if (!q) return item.priority || 0;
    const hay = norm([item.title,item.category,item.summary,item.url,(item.keywords||[]).join(' '),item.text].join(' '));
    if (!hay.includes(q)) return -1;
    let value = item.priority || 0;
    if (norm(item.title).includes(q)) value += 40;
    if (norm(item.url).includes(q)) value += 20;
    if (norm(item.category).includes(q)) value += 15;
    return value;
  }
  function matchFilter(item) {
    if (state.filter === 'all') return true;
    const text = [item.category,item.scope,item.module,(item.tags||[]).join(' ')].join(' ');
    return text.includes(state.filter);
  }
  function currentList() {
    const q = norm(state.query.trim());
    return state.docs.map(item => ({item, score: score(item, q)}))
      .filter(row => row.score >= 0 && matchFilter(row.item))
      .sort((a,b) => b.score - a.score || String(a.item.title).localeCompare(String(b.item.title), 'zh-Hans-CN'))
      .map(row => row.item);
  }
  function card(item) {
    return `<article class="result-card"><strong><a href="${item.url}">${item.title}</a></strong><p>${item.summary || ''}</p><div class="result-meta"><span>${item.category}</span><span>${item.type}</span><span>${item.updated || ''}</span><span>${item.url}</span></div></article>`;
  }
  function docCard(item) {
    return `<article class="doc-card"><strong><a href="${item.url}">${item.title}</a></strong><p>${item.summary || ''}</p><div class="result-meta"><span>${item.category}</span><span>${item.scope || ''}</span><span>${item.updated || ''}</span><span>${item.url}</span></div></article>`;
  }
  function render() {
    const list = currentList();
    document.querySelectorAll('[data-search-results]').forEach(el => {
      el.innerHTML = list.slice(0, 12).map(card).join('') || '<p>没有匹配结果。</p>';
    });
    document.querySelectorAll('[data-doc-list]').forEach(el => {
      el.innerHTML = list.map(docCard).join('') || '<p>没有匹配结果。</p>';
    });
    document.querySelectorAll('[data-doc-count]').forEach(el => {
      el.textContent = `${list.length} / ${state.docs.length} 个条目`;
    });
  }
  function setFilter(value) {
    state.filter = value || 'all';
    document.querySelectorAll('[data-filter]').forEach(btn => btn.classList.toggle('is-active', btn.dataset.filter === state.filter));
    render();
  }
  fetch('search-index.json').then(r => r.json()).then(data => {
    state.docs = data;
    const q = params.get('q') || '';
    const cat = params.get('cat') || '';
    document.querySelectorAll('[data-search-input]').forEach(input => { input.value = q; });
    state.query = q;
    if (cat) state.filter = cat;
    render();
    setFilter(state.filter);
  }).catch(() => {
    document.querySelectorAll('[data-search-results],[data-doc-list]').forEach(el => el.innerHTML = '<p>搜索索引加载失败。</p>');
  });
  document.addEventListener('input', (event) => {
    if (event.target.matches('[data-search-input]')) {
      state.query = event.target.value;
      render();
    }
  });
  document.addEventListener('click', (event) => {
    const filter = event.target.closest('[data-filter]');
    if (filter) setFilter(filter.dataset.filter);
    if (event.target.matches('[data-search-clear]')) {
      document.querySelectorAll('[data-search-input]').forEach(input => input.value = '');
      state.query = '';
      render();
    }
  });
  document.addEventListener('keydown', (event) => {
    if (event.key === 'Enter' && event.target.matches('[data-search-input]')) {
      const first = currentList()[0];
      if (first) location.href = first.url;
    }
    if (event.key === 'Escape' && event.target.matches('[data-search-input]')) {
      event.target.value = '';
      state.query = '';
      render();
    }
  });
})();
""".strip()


def write_generated_site(records: Sequence[Dict[str, object]]) -> None:
    write_site_file(SITE / "index.html", build_home(records))
    write_site_file(SITE / "flow.html", build_flow_page())
    write_site_file(SITE / "library.html", build_library_page())
    write_site_file(SITE / "deploy.html", build_deploy_page())
    write_site_file(SITE / "assets" / "site.css", build_css())
    write_site_file(SITE / "assets" / "site.js", build_js())
    write_site_file(SITE / "README.md", "# CUBE 静态文档网站\n\n运行 `py -m http.server 8128 -d D:\\CUBE\\site` 后访问 `http://127.0.0.1:8128/`。\n")


def write_search_index(records: Sequence[Dict[str, object]]) -> None:
    write_site_file(SITE / "search-index.json", json.dumps(records, ensure_ascii=False, indent=2))


def validate_links() -> List[Tuple[str, str, str]]:
    broken: List[Tuple[str, str, str]] = []
    for path in SITE.rglob("*.html"):
        rel_path = rel_url(path)
        if any(rel_path.startswith(prefix) for prefix in VENDOR_RAW_HTML_PREFIXES):
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for ref in extract_refs(path, text):
            ref = ref.strip()
            if not ref or ref.startswith("#") or ref.startswith("//") or ref.lower().startswith(SKIP_SCHEMES):
                continue
            parsed = urlparse(ref)
            if parsed.scheme:
                continue
            local = ref.split("#", 1)[0].split("?", 1)[0]
            if not local:
                continue
            local = unquote(local).replace("\\", "/")
            target = (SITE / local.lstrip("/")).resolve() if local.startswith("/") else (path.parent / local).resolve()
            if target.suffix.lower() in SOURCE_CODE_EXTS:
                continue
            if not target.exists():
                broken.append((rel_path, ref, str(target)))
    return broken


def main() -> None:
    ensure_site_reset()
    copied: Set[Path] = set()
    copy_initial_files(copied)
    copy_referenced_files(copied)
    records = build_search_index()
    write_search_index(records)
    write_generated_site(records)
    broken = validate_links()
    print(f"site: {SITE}")
    print(f"copied_files: {len(copied)}")
    print(f"search_records: {len(records)}")
    print(f"broken_links: {len(broken)}")
    for src, ref, target in broken[:50]:
        print(f"BROKEN {src} -> {ref} => {target}")
    if broken:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
