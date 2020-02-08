import sys
from lxml.html import parse, etree

chapter_ids = ["intro", "pend", "acrobot", "simple_legs", "humanoids", "stochastic", "dp", "lqr", "lyapunov", "trajopt", "planning", "feedback_motion_planning", "policy_search", "robust", "output_feedback", "limit_cycles", "contact", "rl_policy_search", "drake", "multibody", "optimization", "playbook"]

# Replaces the text between startstr and endstr (non-inclusive), and replaces it
# with fillstr.
def fill_in_file(filename, startstr, endstr, fillstr):
  f = open(filename, "r")
  s = f.read()
  f.close()

  start = s.find(startstr) + len(startstr)
  end = s.find(endstr, start)

  f = open(filename, "w")
  f.write(s[:start] + fillstr + s[end:])
  f.close()

toc = "\n<h1>Table of Contents</h1>\n"
toc += "<ul>\n"
toc += '  <li><a href="#preface">Preface</a></li>\n'

chapter_num = 1
for id in chapter_ids:
  doc = parse(id+".html").getroot()
  chapter = next(doc.iter('chapter'))
  toc += '  <li><a href="'+id+'.html">Chapter '+str(chapter_num)+': '+chapter.find('h1').text+'</a></li>\n'
  chapter_num += 1
  section_num = 1
  if chapter.find('section') is not None:
    toc += '  <ul>\n'
    for section in chapter.findall('section'):
      hash = "section" + str(section_num)
      if section.get('id') is not None:
        hash = section.get('id')
      toc += '    <li><a href='+id+'.html#'+hash+">"+section.find('h1').text+'</a></li>\n'
      section_num += 1
      if section.find('subsection') is not None:
        toc += '    <ul>\n'
        for subsection in section.findall('subsection'):
          toc += '      <li>'+subsection.find('h1').text+'</li>\n'
        toc += '    </ul>\n'
    toc += '  </ul>\n'

toc += '</ul>\n'

fill_in_file("underactuated.html", '<section id="table_of_contents">','</section>', toc)

# TODO: write chapter navigation (and zap redundant chapters.js)
# TODO: Add parts and appendix