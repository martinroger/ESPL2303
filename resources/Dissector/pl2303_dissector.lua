local pl2303_proto = Proto("pl2303", "Prolific PL2303")

-- Define Fields for our Tree
local f_desc = ProtoField.string("pl2303.desc", "Description")
local f_baud = ProtoField.uint32("pl2303.baud", "Baud Rate", base.DEC)
local f_stat = ProtoField.string("pl2303.status", "Line Status")

pl2303_proto.fields = { f_desc, f_baud, f_stat }

local usb_type = Field.new("usb.bmRequestType")


function pl2303_proto.dissector(buffer, pinfo, tree)
    -- Quit on ZLP
    local length = buffer:len()
    if length<1 then return end
    
    local type_f = usb_type()

    -- Find if this is the interrupt endpoint
    local is_interrupt = string.find(tostring(pinfo.src), "%.1$") or string.find(tostring(pinfo.dst), "%.1$")
    
    if is_interrupt and length >= 9 then
        local subtree = tree:add(pl2303_proto, buffer(), "PL2303 Interrupt Status")
        local status_byte = buffer(8,1):uint()
        
        local dcd = ((status_byte & 0x01) ~= 0) and "DCD:1" or "DCD:0"
        local dsr = ((status_byte & 0x02) ~= 0) and "DSR:1" or "DSR:0"
        local ri =  ((status_byte & 0x08) ~= 0) and "RI:1"  or "RI:0"
        local cts = ((status_byte & 0x80) ~= 0) and "CTS:1" or "CTS:0"
        
        local msg = string.format("Status Update: %s, %s, %s, %s", dcd, dsr, cts, ri)
        pinfo.cols.info:set(msg)
        pinfo.cols.protocol = "PL2303"
        subtree:add(f_stat, buffer(8,1), msg)
        return
    end
    
    
    local msg = "Unknown Transaction"
    if not type_f and (buffer:len() >= 7 ) and (buffer:len() <= 10) and string.find(tostring(pinfo.src), "%.0$") then 
        print("No Valid bmRequestType")
        if (buffer:len()) == 7  then
            local baud = buffer(0,4):le_uint()
            msg = "GET LINE Response"
            msg = msg .. string.format(" (%d baud)", baud)
        end
    elseif type_f then
        print("Valid bmRequestType")
        local b_type = type_f.value
        local b_req = buffer(0,1):uint()
        local wVal = buffer(1, 2):le_uint()
        print(wVal)
        if b_type == 0x40 then msg = "VENDOR WRITE Request"
        elseif b_type == 0xC0 then 
            msg = "VENDOR READ Request"..string.format(" %04X",wVal)
        elseif b_type == 0x21 then
            if b_req == 0x20 then 
                msg = "SET LINE Request" -- Somehow does not trigger
                if buffer:len() >= 10 then
                    local baud = buffer(7,4):le_uint()
                    msg = msg .. string.format(" (%d baud)", baud)
                end
            elseif b_req == 0x22 then 
                local dtr = (wVal & 0x01 ~= 0) and 1 or 0
                local rts = (wVal & 0x02 ~= 0) and 1 or 0
                msg = string.format("SET CONTROL: DTR=%d RTS=%d", dtr, rts)
            elseif b_req == 0x23 then msg = "BREAK Request"
            end
        elseif b_type == 0xA1 then msg = "GET LINE Request"
        end
    else return    
    end

    local subtree = tree:add(pl2303_proto, buffer(), "PL2303 Vendor Details")
    pinfo.cols.info:set(msg)
    pinfo.cols.protocol = "PL2303"
    subtree:add(f_desc, msg)

end

-- Registration
-- DissectorTable.get("usb.control"):add_for_decode_as(pl2303_proto)
-- DissectorTable.get("usb.control"):add(0x00000000, pl2303_proto)
DissectorTable.get("usb.product"):add(0x067b2303, pl2303_proto)
-- Register for Interrupt/Bulk transfers as well
-- DissectorTable.get("usb.bulk"):add(0x00000000, pl2303_proto)
-- DissectorTable.get("usb.interrupt"):add(0x00000000, pl2303_proto)