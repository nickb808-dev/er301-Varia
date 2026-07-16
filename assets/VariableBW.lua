-- VariableBW.lua — ER-301 unit wrapper for Variable BW Filter v0.1.0
--
-- A resonant take on the Serge Variable Bandwidth Filter: a bandpass whose
-- centre FREQ and BANDWIDTH are independently CV-controllable, built from two
-- state-variable filters in series (highpass @ fL → lowpass @ fH).  Unlike the
-- flat original, RESONANCE emphasises both band edges and, at the top of its
-- range, self-oscillates.  Stereo in → stereo out.
--
-- CONTROLS
--   freq   center frequency (V/oct trackable)
--   bw     bandwidth  0 = narrow (tight band) … 1 = wide (~6 octaves)
--   res    resonance  0 = flat/gentle … 1 = self-oscillation at the edges
--   level  output gain (unity = 1)

local app      = app
local Class    = Class or require "Base.Class"
local Unit     = require "Unit"
local GainBias = require "Unit.ViewControl.GainBias"
local Encoder  = require "Encoder"
local SlopeView = require "varia.SlopeView"

local libvaria = require "varia.libvaria"

-- Center-frequency dial map: 20 Hz … ~20.5 kHz in 1/12-octave steps.  The stock
-- "oscFreq" map bottoms out near 0.03 Hz, so most of the encoder travel is
-- sub-audio dead zone; this starts the control right at 20 Hz (audible).
local function centerFreqMap()
  local F0, step = 20.0, 1.0 / 12.0
  local map = app.LUTDialMap(121)                        -- 10 octaves × 12 + 1
  for i = 0, 120 do map:add(F0 * (2 ^ (i * step))) end   -- 20 Hz → 20480 Hz
  return map
end

local VariableBW = Class {}
VariableBW:include(Unit)

function VariableBW:init(args)
  args.title    = "Varia"
  args.mnemonic = "Va"
  Unit.init(self, args)
end

function VariableBW:onLoadGraph(channelCount)
  local flt = self:addObject("flt", libvaria.VariableBW())

  connect(self, "In1", flt, "InL")
  if channelCount > 1 then
    connect(self, "In2", flt, "InR")
  end

  -- Freq — center frequency in Hz, V/oct trackable.
  local freqParam = self:addObject("freqParam", app.GainBias())
  local freqRange = self:addObject("freqRange", app.MinMax())
  freqParam:hardSet("Bias", 500.0)
  connect(freqParam, "Out", freqRange, "In")
  connect(freqParam, "Out", flt, "Freq")
  self:addMonoBranch("freq", freqParam, "In", freqParam, "Out")

  -- Bandwidth [0,1].
  local bwParam = self:addObject("bwParam", app.GainBias())
  local bwRange = self:addObject("bwRange", app.MinMax())
  bwParam:hardSet("Bias", 0.3)
  connect(bwParam, "Out", bwRange, "In")
  connect(bwParam, "Out", flt, "Bandwidth")
  self:addMonoBranch("bw", bwParam, "In", bwParam, "Out")

  -- Resonance [0,1].
  local resParam = self:addObject("resParam", app.GainBias())
  local resRange = self:addObject("resRange", app.MinMax())
  resParam:hardSet("Bias", 0.0)
  connect(resParam, "Out", resRange, "In")
  connect(resParam, "Out", flt, "Resonance")
  self:addMonoBranch("res", resParam, "In", resParam, "Out")

  -- Level [0,2].
  local levelParam = self:addObject("levelParam", app.GainBias())
  local levelRange = self:addObject("levelRange", app.MinMax())
  levelParam:hardSet("Bias", 1.0)
  connect(levelParam, "Out", levelRange, "In")
  connect(levelParam, "Out", flt, "Level")
  self:addMonoBranch("level", levelParam, "In", levelParam, "Out")

  -- Phase [-1,1] — STEREO ONLY.  Wired only in a stereo lane; in a mono lane
  -- the Phase inlet stays unconnected (reads 0), so the sound is unchanged.
  if channelCount > 1 then
    local phaseParam = self:addObject("phaseParam", app.GainBias())
    local phaseRange = self:addObject("phaseRange", app.MinMax())
    phaseParam:hardSet("Bias", 0.0)
    connect(phaseParam, "Out", phaseRange, "In")
    connect(phaseParam, "Out", flt, "Phase")
    self:addMonoBranch("phase", phaseParam, "In", phaseParam, "Out")
  end

  connect(flt, "OutL", self, "Out1")
  if channelCount > 1 then
    connect(flt, "OutR", self, "Out2")
  end
end

function VariableBW:onLoadViews(objects, branches)
  local controls = {}

  controls.freq = GainBias {
    button      = "freq",
    description = "Center Frequency",
    branch      = branches.freq,
    gainbias    = objects.freqParam,
    range       = objects.freqRange,
    biasMap     = centerFreqMap(),
    biasUnits   = app.unitHertz,
    initialBias = 500.0,
    gainMap     = Encoder.getMap("freqGain"),
    scaling     = app.octaveScaling,
  }

  controls.bw = GainBias {
    button      = "bw",
    description = "Bandwidth",
    branch      = branches.bw,
    gainbias    = objects.bwParam,
    range       = objects.bwRange,
    biasMap     = Encoder.getMap("[0,1]"),
    initialBias = 0.3,
    gainMap     = Encoder.getMap("[-1,1]"),
  }

  controls.res = GainBias {
    button      = "res",
    description = "Resonance",
    branch      = branches.res,
    gainbias    = objects.resParam,
    range       = objects.resRange,
    biasMap     = Encoder.getMap("[0,1]"),
    initialBias = 0.0,
    gainMap     = Encoder.getMap("[-1,1]"),
  }

  controls.level = GainBias {
    button      = "level",
    description = "Unity = 1",
    branch      = branches.level,
    gainbias    = objects.levelParam,
    range       = objects.levelRange,
    biasMap     = Encoder.getMap("[0,2]"),
    initialBias = 1.0,
    gainMap     = Encoder.getMap("[-1,1]"),
  }

  -- Live filter-slope phosphor scope (Dirac-style), leading the expanded strip.
  controls.slope = SlopeView {
    name   = "slope",
    filter = objects.flt,
    width  = app.SECTION_PLY,
  }

  local expanded = {"slope", "freq", "bw", "res", "level"}

  -- Phase — stereo-only (phaseParam exists only in a stereo lane).
  if objects.phaseParam then
    controls.phase = GainBias {
      button      = "phase",
      description = "Stereo Peak Phase",
      branch      = branches.phase,
      gainbias    = objects.phaseParam,
      range       = objects.phaseRange,
      biasMap     = Encoder.getMap("[-1,1]"),
      initialBias = 0.0,
      gainMap     = Encoder.getMap("[-1,1]"),
    }
    expanded[#expanded + 1] = "phase"
  end

  local views = { expanded = expanded, collapsed = {} }
  return controls, views
end

return VariableBW
