"""整理 SES 显示分组；保留已有编译/链接文件及每个文件的选项。"""
from collections import Counter
from pathlib import Path
from html import unescape
from xml.sax.saxutils import quoteattr
import os
import re
import sys
import xml.etree.ElementTree as ET

root = Path(sys.argv[1]).resolve()
project = (Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else
           root / 'build/segger_embedded_studio/motor_control.emProject')
s = project.read_text(encoding='utf-8')
file_pattern = re.compile(r'<file\b[^>]*?/\s*>|<file\b[^>]*>.*?</file>', re.S)


def file_name(block):
    return unescape(re.search(r'file_name="([^"]+)"', block).group(1))


def file_path(name):
    return (project.parent / name.replace('\\', '/')).resolve()


def path_key(name):
    return os.path.normcase(str(file_path(name)))


# 只解析文件夹部分：SDK 生成的全局 configuration 可能含重复属性。
# 同时识别 SDK 原始布局、旧脚本布局以及本脚本生成的 src/inc 布局。
folders = []
depth = 0
for token in re.finditer(r'<folder\b[^>]*>|</folder\s*>', s):
    tag = token.group()
    if tag.startswith('</'):
        depth -= 1
        if depth == 0:
            folders.append((start, token.end(), name))
    else:
        if depth == 0:
            start = token.start()
            name = unescape(re.search(r'\bName="([^"]+)"', tag).group(1))
        if tag.rstrip().endswith('/>'):
            if depth == 0:
                folders.append((start, token.end(), name))
        else:
            depth += 1
assert depth == 0, 'Unbalanced SES folders'
managed = [f for f in folders if f[2] in {'application', 'motor', 'core', 'middleware'}]
assert managed and managed[0][2] == 'application', 'Missing application group'
for left, right in zip(managed, managed[1:]):
    assert not s[left[1]:right[0]].strip(), 'Unexpected folder between managed groups'

blocks = []
for begin, end, name in managed:
    node = ET.fromstring(s[begin:end])
    for cfg in node.findall('.//folder/configuration') + node.findall('./configuration'):
        assert set(cfg.attrib) <= {'Name', 'build_intermediate_directory'}, \
            'Custom folder options require manual preservation'
    blocks.extend(file_pattern.findall(s[begin:end]))

buckets = {name: {'src': [], 'inc': []} for name in ('application', 'motor', 'core')}
middleware = {'vofa_debug': [], 'hpm_motor': []}
for block in blocks:
    name = file_name(block)
    normalized = name.replace('\\', '/').lower()
    if '/vofa_debug/' in normalized:
        middleware['vofa_debug'].append(block)
    elif '/hpm_motor/' in normalized:
        middleware['hpm_motor'].append(block)
    else:
        try:
            parts = file_path(name).relative_to(root).parts
        except ValueError:
            parts = ()
        group = {'app': 'application', 'motor': 'motor', 'core': 'core'}.get(
            parts[0].lower() if parts else '', 'application')
        kind = 'inc' if file_path(name).suffix.lower() in {'.h', '.hpp'} else 'src'
        buckets[group][kind].append(block)

# 添加磁盘上已经存在的头文件引用；不会创建或改写任何业务头文件。
seen = {path_key(file_name(block)) for block in file_pattern.findall(s)}
added = 0
for directory, group in (('app', 'application'), ('motor', 'motor'), ('core', 'core')):
    for header in sorted((root / directory / 'inc').rglob('*.h')):
        rel = os.path.relpath(header, project.parent).replace('\\', '/')
        if path_key(rel) not in seen:
            buckets[group]['inc'].append('<file file_name=' + quoteattr(rel) + '/>')
            seen.add(path_key(rel))
            added += 1


def folder(name, content, full_name):
    # 每个源码分组使用独立的中间目录；每文件的编译选项原样保留。
    return ('<folder Name=' + quoteattr(name) + '>\n'
            '<configuration Name="Common" build_intermediate_directory='
            + quoteattr('Output/$(Configuration)/Obj/$(ProjectName)/' + full_name)
            + '/>\n' + '\n'.join(content) + '\n</folder>')


groups = []
for name in ('application', 'motor', 'core'):
    children = [folder(kind, buckets[name][kind], name + '/' + kind)
                for kind in ('src', 'inc')]
    groups.append(folder(name, children, name))
groups.append(folder('middleware', [folder(name, middleware[name], 'middleware/' + name)
                                   for name in ('vofa_debug', 'hpm_motor')], 'middleware'))
result = s[:managed[0][0]] + '\n'.join(groups) + s[managed[-1][1]:]

# 原有文件条目（含文件选项）必须完整保留，允许增添头文件浏览条目。
assert not (Counter(file_pattern.findall(s)) - Counter(file_pattern.findall(result))), \
    'Existing file entries or per-file options were changed'

# SES 会链接工程树中的 .a；移除 Additional Linker Inputs 中同一归档的重复引用。
project_archives = {path_key(file_name(block)) for block in file_pattern.findall(result)
                    if file_name(block).lower().endswith('.a')}
removed = []


def deduplicate_linker_inputs(match):
    kept = []
    seen_archives = set(project_archives)
    for name in match.group(1).split(';'):
        if not name:
            continue
        key = path_key(unescape(name))
        if key in seen_archives:
            removed.append(name)
        else:
            kept.append(name)
            seen_archives.add(key)
    return 'linker_additional_files="' + ';'.join(kept) + '"'


result = re.sub(r'linker_additional_files="([^"\n]*)"', deduplicate_linker_inputs, result)
if result != s:
    project.write_text(result, encoding='utf-8')
print('SES tree organized: application/motor/core each have src/inc; '
      + str(added) + ' existing header references added; '
      + str(len(removed)) + ' duplicate archive inputs removed.')
