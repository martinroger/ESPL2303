local pl2303_proto = Proto("pl2303", "Prolific PL2303")

-- Define Fields for our Tree
local f_desc = ProtoField.string("pl2303.desc", "Description")
local f_baud = ProtoField.uint32("pl2303.baud", "Baud Rate", base.DEC)

pl2303_proto.fields = { f_desc, f_baud }

local usb_type = Field.new("usb.bmRequestType")
local usb_breq = Field.new("usb.setup.bRequest")

function pl2303_proto.dissector(buffer, pinfo, tree)
    local type_f = usb_type()
    local breq_f = usb_breq()

    if not type_f then 
        -- print("No type_f")
        return 
        end
    -- if not breq_f then
    --     print("Frame ",pinfo.number, " no bRequest")
    --     end

    
    local subtree = tree:add(pl2303_proto, buffer(), "PL2303 Vendor Details")
    pinfo.cols.protocol = "PL2303"

    local msg = "Unknown Transaction"
    local b_type = type_f.value
    local b_req = breq_f and breq_f.value or (buffer:len() >= 2 and buffer(0,1):uint() or nil)
    print("b_req: ",b_req)
    print(buffer)
    
    if b_type == 0x40 then msg = "VENDOR WRITE Request"
    elseif b_type == 0xC0 then msg = "VENDOR READ Request"
    elseif b_type == 0x21 then
        if b_req == 0x20 then 
            msg = "SET LINE Request" -- Somehow does not trigger
            if buffer:len() >= 10 then
                local baud = buffer(7,4):le_uint()
                msg = msg .. string.format(" (%d baud)", baud)
            end
        elseif b_req == 0x22 then msg = "SET CONTROL Request"
        elseif b_req == 0x23 then msg = "BREAK Request"
        end
    elseif b_type == 0xA1 then msg = "GET LINE Request"
    end

    -- Update UI
    pinfo.cols.info:set("PL2303: " .. msg)
    subtree:add(f_desc, msg)
end

-- Registration
DissectorTable.get("usb.control"):add_for_decode_as(pl2303_proto)
DissectorTable.get("usb.product"):add(0x067b2303, pl2303_proto)