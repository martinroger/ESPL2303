local pl2303_proto = Proto("pl2303", "Prolific PL2303")

-- Define Fields for our Tree
local f_desc = ProtoField.string("pl2303.desc", "Description")
local f_baud = ProtoField.uint32("pl2303.baud", "Baud Rate", base.DEC)

pl2303_proto.fields = { f_desc, f_baud }

local usb_type     = Field.new("usb.bmRequestType")


function pl2303_proto.dissector(buffer, pinfo, tree)
    local type_f = usb_type()

    if not type_f then 
        print("No type_f")
        return 
        end

    
    local subtree = tree:add(pl2303_proto, buffer(), "PL2303 Vendor Details")
    pinfo.cols.protocol = "PL2303"

    local msg = "Unknown Transaction"
    
    if type_f.value == 0x40 then
        msg = "VENDOR WRITE Request"
    elseif type_f.value == 0xC0 then
        msg = "VENDOR READ Request"
    elseif type_f.value == 0x21 then
        if buffer(0,1) == 0x20 then msg = "SET LINE Request" -- Somehow does not trigger
        elseif buffer(0,1) == 0x22 then msg = "SET CONTROL Request"
        elseif buffer(0,1) == 0x23 then msg = "BREAK Request"
        end
    elseif type_f.value == 0xA1 then msg = "GET LINE Request"
    end

    -- Update UI
    pinfo.cols.info:set("PL2303: " .. msg)
    subtree:add(f_desc, msg)
end

-- Registration
DissectorTable.get("usb.control"):add_for_decode_as(pl2303_proto)
DissectorTable.get("usb.product"):add(0x067b2303, pl2303_proto)