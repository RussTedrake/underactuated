
function getParameterByName(name) {
    name = name.replace(/[\[]/, "\\[").replace(/[\]]/, "\\]");
    var regex = new RegExp("[\\?&]" + name + "=([^&#]*)"),
      results = regex.exec(location.search);
    return results == null ? "" : decodeURIComponent(results[1].replace(/\+/g, " "));
}

function chapterNumberToID(number) {
    if (number<0) { console.error("Bad chapter number"); }
    var chapters = document.getElementsByClassName("chapter");
    if ((number-1) < chapters.length) {
        return chapters[number-1].id;
    }
    console.error("Bad chapter number");
}

function chapterIDToNumber(id) {
    id = id.toLowerCase();
    var chapters = document.getElementsByClassName("chapter");
    for (j=0; j<chapters.length; j++) {
        if (chapters[j].id.toLowerCase() === id) {
            return j+1;
        }
    }
    console.error("Bad chapter ID");
    return -1;
}

function setPlatform(is_mac) {  // else display linux
    if (is_mac) {
        to_show = document.getElementsByClassName("mac");
        to_hide = document.getElementsByClassName("linux");
    } else {
        to_show = document.getElementsByClassName("linux");
        to_hide = document.getElementsByClassName("mac");
    }
    for (i=0; i<to_show.length; i++) {
        to_show[i].style.display = "inline";
    }
    for (i=0; i<to_hide.length; i++) {
        to_hide[i].style.display = "none";
    }
}

function revealChapters() {
    // generate table of contents
    {
        var i,j;

        var toc = "<h1>Table of Contents</h1>\n<ul>\n<li><a href=#preface>Preface</a></li>\n";
        var chapters = document.getElementsByClassName("chapter");
        var app = document.getElementsByTagName("appendix");
        var start_appendix = chapters.length - app[0].getElementsByClassName("chapter").length;
        var current_part = chapters[0].parentNode;
        var part_number = 1;

        var list_of_drake_examples = "";
        var bib = new Map();

        // Set platform to linux unless I detect mac.
        setPlatform(navigator.appVersion.indexOf("Mac")!=1);

        for (i = 0; i < chapters.length; i++) {
            var titles=chapters[i].getElementsByTagName("h1");
            if (!chapters[i].id) {
                chapters[i].id = "chap"+(i+1);
            }
//            toc = toc + "<li><a href=#"+chapters[i].id+">";
            if (chapters[i].parentNode != current_part) {
                current_part = chapters[i].parentNode;
                part_number += 1;
                if (current_part.className=="part") {
                    toc = toc + "<p style=\"margin-bottom: 0; text-decoration: underline; font-variant: small-caps;\"><b>" + current_part.getAttribute("title") + "</b></p>\n";
                }
            }

//            toc = toc + chapters[i].parentNode.tagName.toLowerCase();
            var chapter_link = "<li><a href=underactuated.html?chapter="+chapterNumberToID(i+1)+">";
            if (chapterNumberToID(i+1) === "bib") {
                chapter_link = "<p/>" + chapter_link;
            } else if (chapters[i].parentNode.tagName.toLowerCase() ==
              "appendix") {
                chapter_link += "Appendix " +
                  String.fromCharCode(64 + i+1-start_appendix) + ": ";
            } else {
                chapter_link += "Chapter " + (i+1) + ": ";
            }
            chapter_link += titles[0].innerHTML + "</a>\n";
            toc += chapter_link;

            var sections = chapters[i].getElementsByTagName("section");
            var level=0;
            for (j=0; j<sections.length; j++) {
                if (sections[j].getAttribute("data-type")=="sect1") {
                    while (level<1) { toc += "<ul>\n"; level += 1; }
                    while (level>1) { toc += "</ul>\n"; level -= 1; }
                } else if (sections[j].getAttribute("data-type")=="sect2") {
                    while (level<2) { toc += "<ul>\n"; level += 1; }
                    while (level>2) { toc += "</ul>\n"; level -= 1; }
                } /*
              else if (sections[j].getAttribute("data-type")=="sect3") {
                while (level<3) { toc += "<ul>\n"; level += 1; }
                while (level>3) { toc += "</ul>\n"; level -= 1; }
              } */
                else { continue; }
                toc += "<li>" + sections[j].getElementsByTagName("h1")[0].innerHTML + "</li>\n";
            }
            while (level>0) { toc += "</ul>\n"; level -= 1; }

            toc += "</li>\n";

            // TODO: only do this work if I'm actually rendering the output (current_chapter id === drake)
            if (true) { // then I'm writing the drake example list, too
                var examples = chapters[i].getElementsByClassName("drake");
                if (examples.length>0) {
                    list_of_drake_examples += chapter_link + "<ul>";
                    for (j=0; j<examples.length; j++) {
                        var name = examples[j].getElementsByTagName("h1");
                        if (name.length>0) {
                            list_of_drake_examples += "<li>"+name[0].innerHTML+"</li>";
                        }
                    }
                    list_of_drake_examples += "</ul>\n";
                }

            }

        }

        toc = toc + "</ul>\n";
        document.getElementById("table_of_contents").innerHTML = toc;
    }

    var elibTags = document.getElementsByTagName('elib');
    for (i = 0; i < elibTags.length; i++) {
        var c = elibTags[i].innerHTML;
        var str = "";
        var index;
        c.split("+").forEach(function(b) {
            if (bib.has(b)) {
                index = bib.get(b);
            } else {
                index = bib.size+1;
                bib.set(b, index);
            }
            if (str.length > 0) {
                str += ", ";
            }
            str += '<a target="_blank" href="http://groups.csail.mit.edu/locomotion/elib.cgi?b=' + b + '">' + index + '</a>';
        });
        elibTags[i].innerHTML = '[' + str + ']';
    }

    MathJax.Hub.Queue(["Typeset",MathJax.Hub,"mathjax_setup"]);

    var current_chapter = getParameterByName("chapter");

    if (current_chapter) {
        if (isNaN(current_chapter)) {
            current_chapter = chapterIDToNumber(current_chapter);
        } else {
            current_chapter = Number(current_chapter);
        }

        document.getElementById("preface").style.display = "none";
        document.getElementById("table_of_contents").style.display = "none";
        //document.getElementById("debug_output").innerHTML="displaying only chapter " + chapter + " of " + chapters.length;
        for (i = 0; i < chapters.length; i++) {
            if ((i+1) != current_chapter) {
                chapters[i].style.display = "none";
            } else {
                chapters[i].style.display = "inline";

                // load images/videos for this chapter only
                // TODO(russt): consider adding a lint test to make sure all
                // relevant elements are typed in as data-src instead of src.
                var images = chapters[i].getElementsByTagName("img");
                for (j=0; j<images.length; j++) {
                    if (images[j].hasAttribute("data-src")) {
                        images[j].setAttribute("src",images[j].getAttribute("data-src"));
                    }
                }
                // load images/videos for this chapter only
                var videos = chapters[i].getElementsByTagName("video");
                for (j=0; j<videos.length; j++) {
                    if (videos[j].hasAttribute("data-src")) {
                        videos[j].setAttribute("src",videos[j].getAttribute("data-src"));
                    }
                    var extra_sources = videos[j].getElementsByTagName("source");
                    for (k=0; k<extra_sources.length; k++) {
                        if (extra_sources[k].hasAttribute("data-src")) {
                            extra_sources[k].setAttribute("src", extra_sources[k].getAttribute("data-src"));
                        }
                    }
                }
                var iframes = chapters[i].getElementsByTagName("iframe");
                for (j=0; j<iframes.length; j++) {
                    if (iframes[j].hasAttribute("data-src")) {
                        iframes[j].setAttribute("src",iframes[j].getAttribute("data-src"));
                    }
                }

                var codeTags = chapters[i].getElementsByTagName('pysrcinclude');
                for (j = 0; j < codeTags.length; j++) {
                    var file = 'src/' + codeTags[j].innerHTML;
                    var xhttp = new XMLHttpRequest();
                    xhttp.onreadystatechange = ( function (j) { return function() {
                        if (this.readyState == 4 && this.status == 200) {
                            var text = hljs.highlight('python',this.responseText,true)
                            codeTags[j].innerHTML =
                              '<div><pre><code class="python">'+text.value+'</code></pre></div>';
                        }
                    } })(j);
                    xhttp.overrideMimeType('text/plain');
                    xhttp.open('GET', file, true);
                    // note: only works for files in or below current directory http://jquery-howto.blogspot.com/2008/12/access-to-restricted-uri-denied-code.html
                    xhttp.send();
                }

                var pysrcTags = chapters[i].getElementsByTagName('pysrc');
                for (j = 0; j < pysrcTags.length; j++) {
                    var file = pysrcTags[j].innerHTML;
                    // TODO(russt): Consider checking that the file exists.
                    var tmp =
                      '<p><pre style="margin-left:6px; display:inline"><code>python <a target="' + file +'" href="src/'+ file + '">' + file + '</a>';
                    if (pysrcTags[j].hasAttribute("args")) {
                        tmp += ' '+ pysrcTags[j].getAttribute("args");
                    }
                    pysrcTags[j].innerHTML = tmp + '</code></pre>';
                    pysrcTags[j].innerHTML +=
                      '<sidenote><a style="font-size:8pt; margin-left:50%" target="scratchpad" href="https://colab.research.google.com/github/RussTedrake/underactuated/blob/master/src/underactuated_scratchpad.ipynb">' +
                      'Colab scratchpad</a></sidenote></p>';
                }

                var jupyterTags = chapters[i].getElementsByTagName('jupyter');
                for (j = 0; j < jupyterTags.length; j++) {
                    var file = jupyterTags[j].innerHTML;
                    // TODO(russt): Consider checking that the file exists.
                    var tmp =
                      '<p><pre style="margin-left:6px; display:inline"><code>jupyter notebook <a target="' + file + '" href="http://github.com/RussTedrake/underactuated/blob/master/src/' + file + '">src/'
                      + file + '</a>';
                    jupyterTags[j].innerHTML = tmp + '</code></pre>\n' +
                      '<a target="' + file + '_colab" href="https://colab.research.google.com/github/RussTedrake/underactuated/blob/master/src/' + file + '">\n' +
                      '  <img src="https://colab.research.google.com/assets/colab-badge.svg" alt="Open In Colab"/></a></p>';
                }

//              document.getElementById("debug_output").innerHTML="got here";
                var display_num = i;
                if (i>=start_appendix) {
                    display_num=i-start_appendix;
                }
                chapters[i].style.counterReset = "chapter " + display_num + " sect1 example_counter theorem algorithm figure";
                MathJax.Hub.Queue(["Typeset",MathJax.Hub,chapters[i]]);
                var nav = "\n<table style=\"width:100%;\"><tr style=\"width:100%\">";
                nav += "<td style=\"width:33%;text-align:left;\">";
                if (i>0) {
                    nav+="<a href=underactuated.html?chapter=" +
                      chapterNumberToID(current_chapter-1) +
                      ">Previous chapter</a>";
                }
                nav += "</td><td style=\"width:33%;text-align:center;\"><a href=underactuated.html>Table of contents</a></td><td style=\"width:33%;text-align:right;\">";
                if ((i+1)<chapters.length) {
                    nav += "<a href=underactuated.html?chapter=" +
                      chapterNumberToID(current_chapter+1) +
                      ">Next chapter</a>";
                }
                nav += "</td></tr></table>\n";
                chapters[i].innerHTML = nav + chapters[i].innerHTML + nav;
            }
        }
        if (chapters[current_chapter-1].id === "drake") {
            // then i'm in the drake appendix, so fill in that content

            // read drake_version.json and set binaries info
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = ( function (j) { return function() {
                if (this.readyState == 4 && this.status == 200) {
                    var data = JSON.parse(this.responseText);

                    document.getElementById("drake-linux-binaries").innerHTML =
                      data.base_url + data.build + "/drake-" + data.version +
                      "-xenial.tar.gz";
                    document.getElementById("drake-mac-binaries").innerHTML =
                      data.base_url + data.build + "/drake-" + data.mac_version
                      + "-mac.tar.gz";
                }
            } })(j);
            xhttp.overrideMimeType('text/plain');
            xhttp.open('GET', 'drake_version.json', true);
            xhttp.send();

            document.getElementById("list_of_drake_examples").innerHTML =
              "<ul>" + list_of_drake_examples + "</ul>";

        }
        if (chapters[current_chapter-1].id === "bib") {
            // TODO(russt): Embed the bibliography in the page (and make sure the reference numbers match up).
            bibtags = "";
            function addbib(value, key) {
                bibtags += key + "+";
            }
            bib.forEach(addbib);
            bibtags = bibtags.slice(0, -1);

            document.getElementById("bibliography").innerHTML =
              "<p>I'm working on a satisfying solution for this.  For now, <a href=http://groups.csail.mit.edu/locomotion/elib.cgi?b=" +
              bibtags + ">click here</a>.</p>";
        }
    }

    var drakeTags = document.getElementsByTagName('drake');
    for (j = 0; j < drakeTags.length; j++) {
        drakeTags[j].innerHTML =
          '<a style="font-variant:small-caps; text-decoration:none;" href="http://drake.mit.edu">Drake</a>';
    }
}
