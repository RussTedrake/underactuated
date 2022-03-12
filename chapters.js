// TODO: Write this in install_html_metadata to keep it in sync with
// Deepnote.json
var deepnote = {
  "intro": "52e7e101-429f-4aef-a373-e4cca7980cfe",
  "pend": "314062d5-b839-4089-b02f-6c21e42e9581",
  "acrobot": "096cffe7-416e-4d51-a471-5fd526ec8fab",
  "simple_legs": "57670718-9faf-4835-a0bd-ed2142d83498",
  "stochastic": "6dcd0780-e1ef-4e2a-9038-de1dc2e8d684",
  "dp": "526ff99b-f112-4247-9b0b-c52f0f88d6ce",
  "lqr": "05eb768f-556f-4673-9f82-b8e681c9339e",
  "lyapunov": "e5ec0aeb-d006-4689-a009-180923e76318",
  "trajopt": "05031f8c-3586-4b47-be79-7f4893cf2f9d",
  "policy_search": "580a0aa5-3a4d-4c74-a50f-aeda83251fdc",
  "limit_cycles": "b03c5019-403b-4693-8305-f864177e89c3",
  "contact": "41a6fb3f-9ceb-4752-883b-7dbcea779f49",
  "sysid": "8f8d3fd5-f29b-43b9-a0d4-4cee1c8b244e",
  "optimization": "4266eff1-41ee-434a-aa62-0f5a1243ecd2"
}

function notebook_header(chapter) {
  if (chapter in deepnote) {
    return `<a href="https://deepnote.com/project/${deepnote[chapter]}/%2F${chapter}.ipynb" style="float:right; margin-top:20px; margin-bottom:-100px;background:white;border:0;" target="${chapter}">
    <img src="https://deepnote.com/buttons/launch-in-deepnote-white.svg"></a>
    <div style="clear:right;"></div>`;
  }
  return "";
}

function notebook_link(chapter, notebook=chapter) {
  if (chapter in deepnote) {
    return `<p><a href="https://deepnote.com/project/${deepnote[chapter]}/%2F${notebook}.ipynb" style="background:none; border:none;" target="${chapter}">  <img src="https://deepnote.com/buttons/launch-in-deepnote-white.svg"></a></p>`;
  }
  return "";  
}