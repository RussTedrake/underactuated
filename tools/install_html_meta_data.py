import sys
from lxml.html import parse, etree

chapter_ids = ["intro", "pend", "acrobot", "simple_legs", "humanoids", "stochastic", "dp", "lqr", "lyapunov", "trajopt", "planning", "feedback_motion_planning", "policy_search", "robust", "output_feedback", "limit_cycles", "contact", "rl_policy_search", "drake", "multibody", "optimization", "playbook"]

underactuated = parse('underactuated.html').getroot()
toc = next(underactuated.iter('section'))
assert(toc.get('id') == 'table_of_contents')
toc.clear()
toc.set('id', 'table_of_contents')

etree.SubElement(toc, 'h1').text = "Table of Contents"
chapters = etree.SubElement(toc, 'ul')
etree.SubElement(chapters, 'li').text = 'Preface'

for id in chapter_ids:
  doc = parse(id+".html").getroot()
  chapter = next(doc.iter('chapter'))
  li = etree.SubElement(chapters, 'li')
  a = etree.SubElement(li, 'a')
  a.set('href',id+'.html')
  a.text = chapter.find('h1').text
  if chapter.find('section') is not None:
    section_ul = etree.SubElement(li, 'ul')
    for section in chapter.findall('section'):
      etree.SubElement(section_ul, 'li').text = section.find('h1').text

etree.ElementTree(underactuated).write("underactuated.html", pretty_print=True)