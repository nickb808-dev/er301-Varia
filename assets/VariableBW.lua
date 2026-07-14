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
    biasMap     = Encoder.getMap("oscFreq"),
    biasUnits   = app.unitHertz,
    initialBias = 500.0,
    gainMap     = Encoder.getMap("freqGain"),
    scaling     = app.octaveScaling,
  }

  controls.bw = GainBias {
    button      = "bw",
    description = "Bandwidth  narrow ‹ › wide",
    branch      = branches.bw,
    gainbias    = objects.bwParam,
    range       = objects.bwRange,
    biasMap     = Encoder.getMap("[0,1]"),
    initialBias = 0.3,
    gainMap     = Encoder.getMap("[-1,1]"),
  }

  controls.res = GainBias {
    button      = "res",
    description = "Resonance  flat ‹ › self-oscillate (edges)",
    branch      = branches.res,
    gainbias    = objects.resParam,
    range       = objects.resRange,
    biasMap     = Encoder.getMap("[0,1]"),
    initialBias = 0.0,
    gainMap     = Encoder.getMap("[-1,1]"),
  }

  controls.level = GainBias {
    button      = "level",
    description = "Output Level (unity = 1)",
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

  local views = { expanded = {"slope", "freq", "bw", "res", "level"}, collapsed = {} }
  return controls, views
end

return VariableBW
