<script lang="ts">
  import { createEventDispatcher } from "svelte";
  import type { BallPosition, OptimizerWeights, OptimizerSettings } from "../../utils/ballOptimizer";
  import { DEFAULT_OPTIMIZER_WEIGHTS, DEFAULT_OPTIMIZER_SETTINGS } from "../../utils/ballOptimizer";
  import type * as d3 from "d3";

  export let x: d3.ScaleLinear<number, number, number>;
  export let y: d3.ScaleLinear<number, number, number>;
  export let twoElement: HTMLDivElement;

  const BALL_RADIUS_INCHES = 2.5;

  const dispatch = createEventDispatcher<{
    generatePath: {
      balls: BallPosition[];
      weights: OptimizerWeights;
      optSettings: OptimizerSettings;
      farMode: boolean;
    };
  }>();

  export let balls: BallPosition[] = [
    { id: "ball-1", x: 36, y: 36 },
    { id: "ball-2", x: 72, y: 108 },
    { id: "ball-3", x: 108, y: 60 },
  ];

  let weights: OptimizerWeights = { ...DEFAULT_OPTIMIZER_WEIGHTS };
  let optSettings: OptimizerSettings = { ...DEFAULT_OPTIMIZER_SETTINGS };
  let panelOpen = false;
  let farMode = false;

  let draggingId: string | null = null;
  let dragOffset = { x: 0, y: 0 };

  function handleMouseDown(e: MouseEvent, id: string) {
    e.stopPropagation();
    draggingId = id;
    const ball = balls.find(b => b.id === id)!;
    const rect = twoElement.getBoundingClientRect();
    const mouseInchX = x.invert(e.clientX - rect.left);
    const mouseInchY = y.invert(e.clientY - rect.top);
    dragOffset = { x: ball.x - mouseInchX, y: ball.y - mouseInchY };
  }

  function handleMouseMove(e: MouseEvent) {
    if (!draggingId || !twoElement) return;
    const rect = twoElement.getBoundingClientRect();
    const rawX = x.invert(e.clientX - rect.left) + dragOffset.x;
    const rawY = y.invert(e.clientY - rect.top) + dragOffset.y;
    const fs = optSettings.fieldSize;
    const clamped = {
      x: Math.max(0, Math.min(fs, rawX)),
      y: Math.max(0, Math.min(fs, rawY)),
    };
    balls = balls.map(b => b.id === draggingId ? { ...b, ...clamped } : b);
  }

  function handleMouseUp() {
    draggingId = null;
    dragOffset = { x: 0, y: 0 };
  }

  function onGenerate() {
    dispatch("generatePath", { balls, weights, optSettings, farMode });
  }

  // Convert inch radius to pixels
  $: ballRadiusPx = x(BALL_RADIUS_INCHES) - x(0);
</script>

<svelte:window on:mousemove={handleMouseMove} on:mouseup={handleMouseUp} />

<!-- Draggable balls rendered as SVG over the field -->
<svg
  class="absolute top-0 left-0 w-full h-full pointer-events-none"
  style="z-index: 35;"
>
  {#each balls as ball (ball.id)}
    <!-- Shadow for depth -->
    <circle
      cx={x(ball.x) + 1}
      cy={y(ball.y) + 1}
      r={ballRadiusPx}
      fill="rgba(0,0,0,0.3)"
      class="pointer-events-none"
    />
    <!-- Ball -->
    <circle
      cx={x(ball.x)}
      cy={y(ball.y)}
      r={ballRadiusPx}
      fill="#1a1a1a"
      stroke="#444"
      stroke-width="1.5"
      class="pointer-events-auto cursor-grab active:cursor-grabbing"
      role="button"
      tabindex="0"
      aria-label="Drag ball {ball.id}"
      on:mousedown={(e) => handleMouseDown(e, ball.id)}
    />
    <!-- White dot highlight -->
    <circle
      cx={x(ball.x) - ballRadiusPx * 0.3}
      cy={y(ball.y) - ballRadiusPx * 0.3}
      r={ballRadiusPx * 0.25}
      fill="rgba(255,255,255,0.5)"
      class="pointer-events-none"
    />
  {/each}
</svg>

<!-- Collapsible optimizer panel -->
<div
  class="absolute bottom-4 left-4 z-50 bg-neutral-900/95 border border-neutral-700 rounded-xl shadow-2xl backdrop-blur-sm"
  style="min-width: 260px; max-width: 320px;"
>
  <!-- Panel header -->
  <button
    class="flex items-center justify-between w-full px-4 py-3 text-sm font-semibold text-white rounded-xl"
    on:click={() => (panelOpen = !panelOpen)}
  >
    <div class="flex items-center gap-2">
      <!-- Ball icon -->
      <svg viewBox="0 0 20 20" class="size-4 fill-neutral-300">
        <circle cx="10" cy="10" r="8" />
        <path d="M4 10 Q10 4 16 10 Q10 16 4 10" fill="none" stroke="white" stroke-width="1"/>
      </svg>
      <span>Ball Path Optimizer</span>
    </div>
    <svg
      xmlns="http://www.w3.org/2000/svg"
      fill="none"
      viewBox="0 0 24 24"
      stroke-width="2"
      stroke="currentColor"
      class="size-4 transition-transform duration-200 text-neutral-400"
      class:rotate-180={panelOpen}
    >
      <path stroke-linecap="round" stroke-linejoin="round" d="m19.5 8.25-7.5 7.5-7.5-7.5" />
    </svg>
  </button>

  {#if panelOpen}
    <div class="px-4 pb-4 flex flex-col gap-3 border-t border-neutral-700 pt-3">

      <!-- Wall margin -->
      <div>
        <label class="text-xs text-neutral-400 font-medium block mb-1">
          Wall Margin: <span class="text-white">{optSettings.wallMargin}"</span>
        </label>
        <input
          type="range" min="2" max="24" step="1"
          bind:value={optSettings.wallMargin}
          class="w-full h-1.5 rounded accent-purple-500 bg-neutral-700"
        />
      </div>

      <div class="text-xs text-neutral-500 font-semibold uppercase tracking-wider mt-1">
        Objective Weights
      </div>

      <!-- wIntake -->
      <div>
        <label class="text-xs text-neutral-400 block mb-1 flex justify-between">
          <span>Intake Proximity</span>
          <span class="text-white">{weights.wIntake.toFixed(2)}</span>
        </label>
        <input
          type="range" min="0" max="3" step="0.05"
          bind:value={weights.wIntake}
          class="w-full h-1.5 rounded accent-blue-500 bg-neutral-700"
        />
      </div>

      <!-- wHeading -->
      <div>
        <label class="text-xs text-neutral-400 block mb-1 flex justify-between">
          <span>Heading Alignment</span>
          <span class="text-white">{weights.wHeading.toFixed(2)}</span>
        </label>
        <input
          type="range" min="0" max="3" step="0.05"
          bind:value={weights.wHeading}
          class="w-full h-1.5 rounded accent-green-500 bg-neutral-700"
        />
      </div>

      <!-- wCurvature -->
      <div>
        <label class="text-xs text-neutral-400 block mb-1 flex justify-between">
          <span>Minimize Curvature</span>
          <span class="text-white">{weights.wCurvature.toFixed(2)}</span>
        </label>
        <input
          type="range" min="0" max="3" step="0.05"
          bind:value={weights.wCurvature}
          class="w-full h-1.5 rounded accent-yellow-500 bg-neutral-700"
        />
      </div>

      <!-- wCentripetal -->
      <div>
        <label class="text-xs text-neutral-400 block mb-1 flex justify-between">
          <span>Centripetal Accel</span>
          <span class="text-white">{weights.wCentripetal.toFixed(2)}</span>
        </label>
        <input
          type="range" min="0" max="3" step="0.05"
          bind:value={weights.wCentripetal}
          class="w-full h-1.5 rounded accent-red-500 bg-neutral-700"
        />
      </div>

      <!-- Tangent scale -->
      <div>
        <label class="text-xs text-neutral-400 block mb-1 flex justify-between">
          <span>Spline Tightness</span>
          <span class="text-white">{optSettings.tangentScale.toFixed(2)}</span>
        </label>
        <input
          type="range" min="0.1" max="1.5" step="0.05"
          bind:value={optSettings.tangentScale}
          class="w-full h-1.5 rounded accent-purple-400 bg-neutral-700"
        />
      </div>

      <!-- FAR mode toggle -->
      <div class="flex items-center justify-between">
        <label class="text-xs text-neutral-400 font-medium">FAR Mode (Red)</label>
        <button
          on:click={() => farMode = !farMode}
          class="px-3 py-1 text-xs font-semibold rounded-full border transition-colors duration-150 {farMode
            ? 'bg-orange-500 text-white border-orange-500'
            : 'bg-neutral-700 text-neutral-300 border-neutral-600'}"
        >
          {farMode ? 'FAR ON' : 'FAR OFF'}
        </button>
      </div>

      <!-- Generate button -->
      <button
        on:click={onGenerate}
        class="mt-1 w-full py-2 px-3 bg-purple-600 hover:bg-purple-500 active:bg-purple-700 text-white text-sm font-semibold rounded-lg transition-colors duration-150 flex items-center justify-center gap-2 shadow-md"
      >
        <svg viewBox="0 0 20 20" class="size-4 fill-white">
          <path d="M10 2a8 8 0 100 16A8 8 0 0010 2zm0 3a1 1 0 011 1v3h3a1 1 0 010 2h-3v3a1 1 0 01-2 0v-3H6a1 1 0 010-2h3V6a1 1 0 011-1z"/>
        </svg>
        Generate Optimal Path
      </button>

      <p class="text-[10px] text-neutral-600 text-center leading-tight">
        Drag black balls on field to set positions. Path replaces current path.
      </p>
    </div>
  {/if}
</div>