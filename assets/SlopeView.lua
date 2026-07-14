-- SlopeView — display-only ViewControl hosting Varia's filter-slope phosphor
-- scope. Draws the live variable-BP magnitude response (X = log frequency,
-- Y = dB), trailing as Freq / Bandwidth / Resonance move.  Modeled on Dirac's
-- GrainFieldView (one control-width phosphor viz, no editable parameters).

local app = app
local Class = require "Base.Class"
local ViewControl = require "Unit.ViewControl"
local libvaria = require "varia.libvaria"
local ply = app.SECTION_PLY

local SlopeView = Class {}
SlopeView:include(ViewControl)

function SlopeView:init(args)
  ViewControl.init(self, args.name or "slope")
  self:setClassName("varia.SlopeView")
  local flt = args.filter or app.logError("%s.init: filter is missing.", self)
  local width = args.width or ply   -- one control-width slot, like Dirac's field

  self.slope = libvaria.SlopeGraphic(0, 0, width, 64)
  self.slope:follow(flt)

  local graphic = app.Graphic(0, 0, width, 64)
  graphic:addChild(self.slope)
  self:setControlGraphic(graphic)
  self:setMainCursorController(self.slope)

  for i = 1, (width // ply) do
    self:addSpotDescriptor{ center = (i - 0.5) * ply }
  end

  -- Sub display: title (no editable parameters).
  self.subGraphic = app.Graphic(0, 0, 128, 64)
  local label = app.Label("Varia — filter response", 10)
  label:fitToText(0)
  label:setCenter(64, 32)
  self.subGraphic:addChild(label)
end

return SlopeView
