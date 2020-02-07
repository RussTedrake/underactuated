var chapter_ids = ["intro", "pend", "acrobot", "simple_legs", "humanoids"];

function chapterNumberToID(number) {
  if (number<0) { console.error("Bad chapter number"); }
  if ((number-1) < chapter_ids.length) {
      return chapter_ids[number-1];
  }
  console.error("Bad chapter number");
}

function chapterIDToNumber(id) {
  id = id.toLowerCase();
  for (j=0; j<chapter_ids.length; j++) {
      if (chapter_ids[j].toLowerCase() === id) {
          return j+1;
      }
  }
  console.error("Bad chapter ID");
  return -1;
}

function loadChapter(chapter_id)  {
  var chapter_num = chapterIDToNumber(chapter_id);
  var chapter = document.getElementsByClassName("chapter")[0];

  chapter.style.counterReset = "chapter " + chapter_num;

  // Make sure all sections have ids and links.
  var sections = chapter.getElementsByTagName("section");
  for (j=0; j<sections.length; j++) {
      if (!sections[j].id) {
          sections[j].id = "section" + (j+1);
      }
      sections[j].getElementsByTagName("h1")[0].innerHTML = "<a href=#" + sections[j].id + ">" + sections[j].getElementsByTagName("h1")[0].innerHTML + "</a>";
  }

  // Make sure all examples have ids and links.
  var examples = chapter.getElementsByTagName("example");
  for (j=0; j<examples.length; j++) {
      if (!examples[j].id) {
          examples[j].id = "example" + (j+1);
      }
      examples[j].getElementsByTagName("h1")[0].innerHTML = "<a href=#" + examples[j].id + ">" + examples[j].getElementsByTagName("h1")[0].innerHTML + "</a>";
  }  
}