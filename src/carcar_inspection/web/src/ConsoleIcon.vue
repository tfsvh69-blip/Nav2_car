<script setup>
// 项目自绘的24×24线条图标，所有平台本地渲染，不依赖字体或远端资源。
defineProps({ name: { type: String, required: true } });
const paths = {
  map: "m3 6 6-3 6 3 6-3v15l-6 3-6-3-6 3Zm6-3v15m6-12v15",
  overview: "M3 3h7v7H3Zm11 0h7v7h-7ZM3 14h7v7H3Zm11 0h7v7h-7Z",
  points:
    "M19 10c0 5-7 11-7 11S5 15 5 10a7 7 0 1 1 14 0ZM9.5 10a2.5 2.5 0 1 0 5 0 2.5 2.5 0 1 0-5 0",
  sketch: "m4 16 12-12 4 4L8 20l-5 1Zm10-10 4 4M4 16l4 4",
  tasks: "M9 6h12M9 12h12M9 18h12M3 5l1 1 2-2M3 11l1 1 2-2M3 17l1 1 2-2",
  running: "m9 5 11 7-11 7ZM3 5v14",
  records: "M8 3h8l4 4v14H4V3Zm8 0v5h4M8 12h8M8 16h5",
  stop: "M7 3h10l4 4v10l-4 4H7l-4-4V7ZM9 9h6v6H9Z",
  connection:
    "M3 8a14 14 0 0 1 18 0M6 12a9 9 0 0 1 12 0m-9 4a4.5 4.5 0 0 1 6 0m-3 4h.01",
  arrow: "M12 19V5m-6 6 6-6 6 6",
  left: "M9 4 4 9l5 5M4 9h9a7 7 0 0 1 7 7v4",
  right: "m15 4 5 5-5 5m5-5h-9a7 7 0 0 0-7 7v4",
  chevron: "m9 5 7 7-7 7",
  camera: "M3 7h4l2-3h6l2 3h4v13H3Zm5 6a4 4 0 1 0 8 0 4 4 0 1 0-8 0",
  check: "m5 12 4 4L19 6",
  plus: "M12 5v14M5 12h14",
  folder: "M3 7V4h6l3 3h9v13H3Z",
  drive: "M7 12 4 18v3m13-9 3 6v3M4 18h16M6 12l2-7h8l2 7ZM7 16h.01M17 16h.01",
  shield: "m12 3 8 3v6c0 5-8 9-8 9s-8-4-8-9V6Zm-4 9 3 3 5-6",
};
</script>

<template>
  <svg
    class="icon"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    stroke-width="1.7"
    stroke-linecap="round"
    stroke-linejoin="round"
    aria-hidden="true"
    focusable="false"
  >
    <path :d="paths[name] || paths.points" />
  </svg>
</template>
