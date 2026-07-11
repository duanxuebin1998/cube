(function () {
  "use strict";

  var controllerKey = "__cubeFlowScrollViewportController";
  var generatedHintText = "此流程图可横向滚动；聚焦后可使用键盘左、右方向键查看完整流程。";
  var previousController = window[controllerKey];
  if (previousController && typeof previousController.destroy === "function") {
    previousController.destroy({ preserveState: true });
  }

  var managed = new Set();
  var hintSequence = 0;
  var resizeFrame = 0;
  var destroyed = false;

  function clamp(value, minimum, maximum) {
    return Math.max(minimum, Math.min(maximum, value));
  }

  function datasetNumber(viewport, key) {
    var value = Number.parseFloat(viewport.dataset[key] || "");
    return Number.isFinite(value) ? value : null;
  }

  function isTableViewport(viewport) {
    return viewport.classList.contains("cube-flow-table__viewport");
  }

  function tableFrame(viewport) {
    var frame = viewport.closest(".cube-flow-table");
    return frame instanceof HTMLElement ? frame : null;
  }

  function tableHint(viewport) {
    var frame = tableFrame(viewport);
    if (!frame) return null;
    var hint = Array.prototype.slice.call(frame.children).find(function (child) {
      return child.classList.contains("cube-flow-table__hint");
    });
    return hint instanceof HTMLElement && hint.id ? hint : null;
  }

  function updateTableHint(viewport, overflow) {
    var frame = tableFrame(viewport);
    var hint = tableHint(viewport);
    if (!frame || !hint) return;
    if (overflow) frame.dataset.cubeTableOverflow = "true";
    else delete frame.dataset.cubeTableOverflow;

    var describedBy = new Set((viewport.getAttribute("aria-describedby") || "").split(/\s+/).filter(Boolean));
    if (overflow) describedBy.add(hint.id);
    else describedBy.delete(hint.id);
    if (describedBy.size) viewport.setAttribute("aria-describedby", Array.from(describedBy).join(" "));
    else viewport.removeAttribute("aria-describedby");
  }

  function nextHintId() {
    var id;
    do {
      hintSequence += 1;
      id = "cube-flow-standalone-keyboard-hint-" + hintSequence;
    } while (document.getElementById(id));
    return id;
  }

  function removeGeneratedHint(viewport) {
    var hintId = viewport.dataset.cubeViewportHintId;
    if (!hintId) return;
    var describedBy = (viewport.getAttribute("aria-describedby") || "").split(/\s+/).filter(function (id) {
      return id && id !== hintId;
    });
    if (describedBy.length) viewport.setAttribute("aria-describedby", describedBy.join(" "));
    else viewport.removeAttribute("aria-describedby");
    var hint = document.getElementById(hintId);
    if (hint && hint.dataset.cubeViewportHint === "true") hint.remove();
    delete viewport.dataset.cubeViewportHintId;
  }

  function ensureGeneratedHint(viewport) {
    var hintId = viewport.dataset.cubeViewportHintId;
    var hint = hintId ? document.getElementById(hintId) : null;
    if (!hint || hint.dataset.cubeViewportHint !== "true") {
      removeGeneratedHint(viewport);
      hint = document.createElement("span");
      hint.id = nextHintId();
      hint.hidden = true;
      hint.dataset.cubeViewportHint = "true";
      hint.textContent = generatedHintText;
      viewport.insertAdjacentElement("afterend", hint);
      viewport.dataset.cubeViewportHintId = hint.id;
    }
    var describedBy = new Set((viewport.getAttribute("aria-describedby") || "").split(/\s+/).filter(Boolean));
    describedBy.add(hint.id);
    viewport.setAttribute("aria-describedby", Array.from(describedBy).join(" "));
  }

  function updateFlowFigure(viewport, overflow) {
    var figure = viewport.closest(".cube-flow-figure");
    if (!(figure instanceof HTMLElement)) return;
    if (overflow) figure.dataset.cubeFlowOverflow = "true";
    else delete figure.dataset.cubeFlowOverflow;
  }

  function initialScrollTarget(viewport) {
    if (isTableViewport(viewport)) return 0;
    var fallback = (viewport.scrollWidth - viewport.clientWidth) / 2;
    var svg = viewport.querySelector("svg");
    var entry = svg && svg.querySelector(".start");
    if (!(svg instanceof SVGSVGElement) || !(entry instanceof SVGGraphicsElement)) return fallback;
    try {
      var viewBox = svg.viewBox.baseVal;
      var bounds = entry.getBBox();
      if (viewBox.width <= 0 || bounds.width <= 0) return fallback;
      var scale = svg.getBoundingClientRect().width / viewBox.width;
      return (bounds.x + bounds.width / 2 - viewBox.x) * scale - viewport.clientWidth / 2;
    } catch (error) {
      return fallback;
    }
  }

  function rememberRatio(viewport, maximum) {
    if (maximum <= 1) {
      delete viewport.dataset.cubeViewportRatio;
      return;
    }
    viewport.dataset.cubeViewportRatio = String(clamp(viewport.scrollLeft, 0, maximum) / maximum);
  }

  function handleScroll(event) {
    var viewport = event.currentTarget;
    if (!(viewport instanceof HTMLElement)) return;
    var previousMaximum = datasetNumber(viewport, "cubeViewportMaximum");
    var currentMaximum = Math.max(0, viewport.scrollWidth - viewport.clientWidth);
    if (previousMaximum === null || Math.abs(previousMaximum - currentMaximum) > 1) return;
    rememberRatio(viewport, previousMaximum);
  }

  function handleKeydown(event) {
    var viewport = event.currentTarget;
    if (!(viewport instanceof HTMLElement) || event.target !== viewport || event.altKey || event.ctrlKey || event.metaKey || event.shiftKey) return;
    var direction = event.key === "ArrowLeft" ? -1 : event.key === "ArrowRight" ? 1 : 0;
    if (!direction) return;
    var maximum = Math.max(0, viewport.scrollWidth - viewport.clientWidth);
    if (maximum <= 1) return;
    var step = Math.max(96, Math.round(viewport.clientWidth * 0.72));
    var target = clamp(viewport.scrollLeft + direction * step, 0, maximum);
    if (target === viewport.scrollLeft) return;
    event.preventDefault();
    viewport.scrollLeft = target;
    rememberRatio(viewport, maximum);
  }

  function clearState(viewport) {
    if (isTableViewport(viewport)) {
      updateTableHint(viewport, true);
      var frame = tableFrame(viewport);
      if (frame) delete frame.dataset.cubeTableOverflow;
    }
    else {
      removeGeneratedHint(viewport);
      updateFlowFigure(viewport, false);
    }
    viewport.tabIndex = 0;
    ["cubeViewportInitialized", "cubeViewportOverflow", "cubeViewportWidth", "cubeViewportMaximum", "cubeViewportRatio"].forEach(function (key) {
      delete viewport.dataset[key];
    });
  }

  function release(viewport, clear) {
    viewport.removeEventListener("keydown", handleKeydown);
    viewport.removeEventListener("scroll", handleScroll);
    managed.delete(viewport);
    if (clear) clearState(viewport);
  }

  function refresh() {
    if (destroyed) return;
    var current = new Set(document.querySelectorAll(".cube-flow__viewport, .cube-flow-table__viewport"));
    managed.forEach(function (viewport) {
      if (!current.has(viewport)) release(viewport, true);
    });

    current.forEach(function (viewport) {
      if (!(viewport instanceof HTMLElement)) return;
      if (!managed.has(viewport)) {
        managed.add(viewport);
        viewport.addEventListener("keydown", handleKeydown);
        viewport.addEventListener("scroll", handleScroll, { passive: true });
      }

      var maximum = Math.max(0, viewport.scrollWidth - viewport.clientWidth);
      var overflow = maximum > 1;
      var initialized = viewport.dataset.cubeViewportInitialized === "true";
      var previouslyOverflowed = viewport.dataset.cubeViewportOverflow === "true";
      var previousMaximum = datasetNumber(viewport, "cubeViewportMaximum");
      var previousRatio = datasetNumber(viewport, "cubeViewportRatio");

      if (!overflow) {
        viewport.removeAttribute("tabindex");
        viewport.scrollLeft = 0;
        delete viewport.dataset.cubeViewportRatio;
        if (isTableViewport(viewport)) updateTableHint(viewport, false);
        else {
          removeGeneratedHint(viewport);
          updateFlowFigure(viewport, false);
        }
      } else {
        viewport.tabIndex = 0;
        if (isTableViewport(viewport)) updateTableHint(viewport, true);
        else {
          ensureGeneratedHint(viewport);
          updateFlowFigure(viewport, true);
        }

        var target = viewport.scrollLeft;
        if (!initialized || !previouslyOverflowed) target = initialScrollTarget(viewport);
        else if (previousMaximum !== null && Math.abs(previousMaximum - maximum) > 1) {
          var ratio = previousRatio !== null ? previousRatio : previousMaximum > 1 ? clamp(viewport.scrollLeft, 0, previousMaximum) / previousMaximum : 0;
          target = ratio * maximum;
        }
        viewport.scrollLeft = clamp(target, 0, maximum);
        rememberRatio(viewport, maximum);
      }

      viewport.dataset.cubeViewportInitialized = "true";
      viewport.dataset.cubeViewportOverflow = String(overflow);
      viewport.dataset.cubeViewportWidth = String(viewport.clientWidth);
      viewport.dataset.cubeViewportMaximum = String(maximum);
    });
  }

  function scheduleRefresh() {
    if (destroyed) return;
    if (resizeFrame) cancelAnimationFrame(resizeFrame);
    resizeFrame = requestAnimationFrame(function () {
      resizeFrame = 0;
      refresh();
    });
  }

  function destroy(options) {
    if (destroyed) return;
    destroyed = true;
    if (resizeFrame) cancelAnimationFrame(resizeFrame);
    window.removeEventListener("resize", scheduleRefresh);
    managed.forEach(function (viewport) {
      release(viewport, !options || options.preserveState !== true);
    });
    if (window[controllerKey] === controller) delete window[controllerKey];
  }

  var controller = { destroy: destroy, refresh: refresh };
  window[controllerKey] = controller;
  window.addEventListener("resize", scheduleRefresh, { passive: true });
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", scheduleRefresh, { once: true });
  else scheduleRefresh();
}());
