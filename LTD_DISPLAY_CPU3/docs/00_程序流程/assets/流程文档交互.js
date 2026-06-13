(function () {
  "use strict";

  function normalise(text) {
    return String(text || "").toLowerCase().replace(/\s+/g, " ").trim();
  }

  function textOf(element) {
    return normalise(element.getAttribute("data-search-text") || element.textContent || "");
  }

  function updateStatus(statusElement, visible, total) {
    if (!statusElement) return;
    statusElement.textContent = "显示 " + visible + " / " + total + " 项";
  }

  function setupLocalFilters() {
    document.querySelectorAll("[data-doc-filter-input]").forEach(function (input) {
      var targetSelector = input.getAttribute("data-filter-target");
      var itemSelector = input.getAttribute("data-filter-item") || "tr, .func-card, .phase";
      var statusSelector = input.getAttribute("data-filter-status");
      var target = targetSelector ? document.querySelector(targetSelector) : document;
      var statusElement = statusSelector ? document.querySelector(statusSelector) : null;
      if (!target) return;

      var items = Array.prototype.slice.call(target.querySelectorAll(itemSelector));
      var apply = function () {
        var query = normalise(input.value);
        var visible = 0;
        items.forEach(function (item) {
          var matched = !query || textOf(item).indexOf(query) !== -1;
          item.classList.toggle("filter-hidden", !matched);
          if (matched) visible += 1;
        });
        updateStatus(statusElement, visible, items.length);
      };

      input.addEventListener("input", apply);
      apply();
    });
  }

  function setupDetailsControls() {
    document.querySelectorAll("[data-expand-all], [data-collapse-all]").forEach(function (button) {
      button.addEventListener("click", function () {
        var targetSelector = button.getAttribute("data-expand-all") || button.getAttribute("data-collapse-all");
        var target = targetSelector ? document.querySelector(targetSelector) : document;
        if (!target) return;
        var open = button.hasAttribute("data-expand-all");
        target.querySelectorAll("details, .func-detail").forEach(function (detail) {
          if (detail.tagName && detail.tagName.toLowerCase() === "details") {
            detail.open = open;
          } else {
            detail.classList.toggle("is-hidden", !open);
          }
        });
      });
    });
  }

  function setupIssueFilters() {
    document.querySelectorAll("[data-issue-filter]").forEach(function (button) {
      button.addEventListener("click", function () {
        var level = button.getAttribute("data-issue-filter");
        var group = button.closest("[data-issue-filter-group]") || document;
        group.querySelectorAll("[data-issue-filter]").forEach(function (peer) {
          peer.classList.toggle("is-active", peer === button);
        });
        document.querySelectorAll("[data-issue-level]").forEach(function (row) {
          var rowLevel = row.getAttribute("data-issue-level");
          row.classList.toggle("issue-hidden", level !== "all" && rowLevel !== level);
        });
      });
    });
  }

  function setupGlobalSearch() {
    var input = document.querySelector("[data-global-search]");
    var output = document.querySelector("[data-global-search-results]");
    var dataElement = document.getElementById("doc-search-data");
    if (!input || !output || !dataElement) return;

    var data = [];
    try {
      data = JSON.parse(dataElement.textContent || "[]");
    } catch (error) {
      output.innerHTML = '<p class="muted">搜索索引解析失败。</p>';
      return;
    }

    var render = function () {
      var query = normalise(input.value);
      output.innerHTML = "";
      if (query.length < 2) {
        output.innerHTML = '<p class="search-hint">输入至少 2 个字符，可以按指令、函数、文件、状态宏、问题关键词查找。</p>';
        return;
      }

      var terms = query.split(" ").filter(Boolean);
      var results = data.filter(function (item) {
        var haystack = normalise([item.type, item.title, item.domain, item.file, item.summary, item.keywords].join(" "));
        return terms.every(function (term) { return haystack.indexOf(term) !== -1; });
      }).slice(0, 80);

      if (!results.length) {
        output.innerHTML = '<p class="muted">没有匹配结果。</p>';
        return;
      }

      var fragment = document.createDocumentFragment();
      results.forEach(function (item) {
        var link = document.createElement("a");
        link.className = "search-result";
        link.href = item.href;
        var title = document.createElement("strong");
        var type = document.createElement("em");
        var meta = document.createElement("span");
        type.textContent = item.type;
        title.appendChild(type);
        title.appendChild(document.createTextNode(item.title));
        meta.textContent = [item.domain, item.file, item.summary].filter(Boolean).join(" | ");
        link.appendChild(title);
        link.appendChild(meta);
        fragment.appendChild(link);
      });
      output.appendChild(fragment);
    };

    input.addEventListener("input", render);
    render();
  }

  document.addEventListener("DOMContentLoaded", function () {
    setupGlobalSearch();
    setupLocalFilters();
    setupDetailsControls();
    setupIssueFilters();
  });
}());
