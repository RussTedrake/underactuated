import sys
from lxml.html import parse, etree

chapter_ids = ["intro", "pend", "acrobot", "simple_legs", "humanoids", "stochastic", "dp", "lqr", "lyapunov", "trajopt", "planning", "feedback_motion_planning", "policy_search", "robust", "output_feedback", "limit_cycles", "contact", "sysid", "estimation", "rl_policy_search", "value_learning", "actor_critic", "drake", "multibody", "optimization", "playbook"]

# key is the first chapter id in the part, value is the name of the part
parts = { 'pend': 'Model Systems', 'dp': 'Nonlinear Planning and Control', 'sysid': 'Estimation and Learning', 'drake': 'Appendix'}

change_detected = False

def get_file_as_string(filename):
  f = open(filename, "r")
  s = f.read()
  f.close()
  return s

def write_file_as_string(filename, s):
  f = open(filename, "w")
  f.write(s)
  f.close()

def replace_string_before(s, before_str, with_str):
  global change_detected
  r = with_str + s[s.find(before_str):]
  change_detected = change_detected or (r != s)
  return r

def replace_string_after(s, after_str, with_str):
  global change_detected
  loc = s.find(after_str) + len(after_str)
  r = s[:loc] + with_str
  change_detected = change_detected or (r != s)
  return r

def replace_string_between(s, start_str, end_str, with_str):
  global change_detected
  index = 0
  while s.find(start_str, index) > 0:
    start = s.find(start_str, index) + len(start_str)
    end = s.find(end_str, start)
    r = s[:start] + with_str + s[end:]
    change_detected = change_detected or (r != s)
    s = r
    index = start + len(with_str)
  return s


header = get_file_as_string("tools/header.html.in")
footer = get_file_as_string("tools/footer.html.in")

chapter_num = 1
for id in chapter_ids:
  filename = id + ".html"
  s = get_file_as_string(filename)

  # Rewrite the header
  s = replace_string_before(s, "<chapter", header)

  # Rewrite the footer
  s = replace_string_after(s, "</chapter>", footer)

  # Update the chapter number
  s = replace_string_between(s, "<chapter", ">", ' style="counter-reset: chapter '+str(chapter_num-1) + '"')

  # Write previous and next chapter logic
  if chapter_num > 1:
    s = replace_string_between(s, '<a class="previous_chapter"', '</a>', ' href='+chapter_ids[chapter_num-2]+'.html>Previous Chapter')
  if chapter_num < len(chapter_ids):
    s = replace_string_between(s, '<a class="next_chapter"', '</a>', ' href='+chapter_ids[chapter_num]+'.html>Next Chapter')

  write_file_as_string(filename, s)

  chapter_num += 1


# Build TOC
toc = "\n<h1>Table of Contents</h1>\n"
toc += "<ul>\n"
toc += '  <li><a href="#preface">Preface</a></li>\n'

chapter_num = 1
for id in chapter_ids:
  filename = id + ".html"

  doc = parse(filename).getroot()
  chapter = next(doc.iter('chapter'))

  # Write the part if this chapter starts a new one.
  if chapter in parts:
    toc += '<p style="margin-bottom: 0; text-decoration: underline; font-variant: small-caps;"><b>' + parts[chapter] + '</b></p>\n'

  toc += '  <li><a href="'+filename+'">Chapter '+str(chapter_num)+': '+chapter.find('h1').text+'</a></li>\n'
  chapter_num += 1
  section_num = 1
  if chapter.find('section') is not None:
    toc += '  <ul>\n'
    for section in chapter.findall('section'):
      hash = "section" + str(section_num)
      if section.get('id') is not None:
        hash = section.get('id')
      toc += '    <li><a href='+filename+'#'+hash+">"+section.find('h1').text+'</a></li>\n'
      section_num += 1
      if section.find('subsection') is not None:
        toc += '    <ul>\n'
        for subsection in section.findall('subsection'):
          toc += '      <li>'+subsection.find('h1').text+'</li>\n'
        toc += '    </ul>\n'
    toc += '  </ul>\n'

toc += '</ul>\n'

s = get_file_as_string("underactuated.html")
s = replace_string_between(s, '<section id="table_of_contents">','</section>', toc)
write_file_as_string("underactuated.html", s)

exit(change_detected)
