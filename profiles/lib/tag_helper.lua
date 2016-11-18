local TagHelper = {}

function TagHelper.directional_key_value(way,is_forward,key)
  local direction
  if is_forward then
    direction = 'forward'
  else
    direction = 'backward'
  end
  local directional_key = key .. ':' .. direction
  local value = way:get_value_by_key(directional_key)
  if value then
    return directional_key, value
  end
  
  value = way:get_value_by_key(key)
  if value then
    return key, value
  end 
end

function TagHelper.directional_value(way,is_forward,key)
  local direction
  if is_forward then
    direction = 'forward'
  else
    direction = 'backward'
  end
  return way:get_value_by_key(key .. ':' .. direction) or way:get_value_by_key(key)
end

-- return [forward,backward] values for a specific tag.
-- e.g. for maxspeed search forward:
--   maxspeed:forward
--   maxspeed
-- and backward:
--   maxspeed:backward
--   maxspeed

function TagHelper.directional_values(way,key)
  local forward = way:get_value_by_key(key .. ':forward')
  local backward = way:get_value_by_key(key .. ':backward')
  
  if forward and backward then
    return forward, backward
  end
  
  local common = way:get_value_by_key(key)
  return forward or common,
         backward or common
end

-- return [forward,backward] values, searching a 
-- prioritized sequence of tags
-- e.g. for the sequence [maxspeed,advisory] search forward:
--   maxspeed:forward
--   maxspeed
--   advisory:forward
--   advisory
-- and for backward:
--   maxspeed:backward
--   maxspeed
--   advisory:backward
--   advisory

function TagHelper.pick_directional_values(way,keys)
  local forward, backward
  for i,key in ipairs(keys) do
    if not forward then
      forward = way:get_value_by_key(key .. ':forward')
    end
    if not backward then
      backward = way:get_value_by_key(key .. ':backward')
    end
    if not forward or not backward then
      local common = way:get_value_by_key(key)
      forward = forward or common
      backward = backward or common
    end
    if forward and backward then
      return forward, backward
    end
  end
  return forward, backward
end

return TagHelper