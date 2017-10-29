/**
 * This class is automatically generated by mig. DO NOT EDIT THIS FILE.
 * This class implements a Java interface to the 'TestSerialMsg'
 * message type.
 */

public class TestSerialMsg extends net.tinyos.message.Message {

    /** The default size of this message type in bytes. */
	//modifed by Hyungjune Lee
    public static final int DEFAULT_MESSAGE_SIZE = 7; 

    /** The Active Message type associated with this message. */
    public static final int AM_TYPE = 6;

    /** Create a new TestSerialMsg of size 2. */
    public TestSerialMsg() {
        super(DEFAULT_MESSAGE_SIZE);
        amTypeSet(AM_TYPE);
    }

    /** Create a new TestSerialMsg of the given data_length. */
    public TestSerialMsg(int data_length) {
        super(data_length);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TestSerialMsg with the given data_length
     * and base offset.
     */
    public TestSerialMsg(int data_length, int base_offset) {
        super(data_length, base_offset);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TestSerialMsg using the given byte array
     * as backing store.
     */
    public TestSerialMsg(byte[] data) {
        super(data);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TestSerialMsg using the given byte array
     * as backing store, with the given base offset.
     */
    public TestSerialMsg(byte[] data, int base_offset) {
        super(data, base_offset);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TestSerialMsg using the given byte array
     * as backing store, with the given base offset and data length.
     */
    public TestSerialMsg(byte[] data, int base_offset, int data_length) {
        super(data, base_offset, data_length);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TestSerialMsg embedded in the given message
     * at the given base offset.
     */
    public TestSerialMsg(net.tinyos.message.Message msg, int base_offset) {
        super(msg, base_offset, DEFAULT_MESSAGE_SIZE);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TestSerialMsg embedded in the given message
     * at the given base offset and length.
     */
    public TestSerialMsg(net.tinyos.message.Message msg, int base_offset, int data_length) {
        super(msg, base_offset, data_length);
        amTypeSet(AM_TYPE);
    }

    /**
    /* Return a String representation of this message. Includes the
     * message type name and the non-indexed field values.
     */
    public String toString() {
      String s = "Message <TestSerialMsg> \n";
      try {
        s += "  [counter=0x"+Long.toHexString(get_counter())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      return s;
    }

    // Message-type-specific access methods appear below.

    /////////////////////////////////////////////////////////
    // Accessor methods for field: counter
    //   Field type: int, unsigned
    //   Offset (bits): 0
    //   Size (bits): 16
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'counter' is signed (false).
     */
    public static boolean isSigned_counter() {
        return false;
    }

    /**
     * Return whether the field 'counter' is an array (false).
     */
    public static boolean isArray_counter() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'counter'
     */
    public static int offset_counter() {
        return (0 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'counter'
     */
    public static int offsetBits_counter() {
        return 0;
    }

    /**
     * Return the value (as a int) of the field 'counter'
     */
    public int get_counter() {
        return (int)getUIntBEElement(offsetBits_counter(), 16);
    }

    /**
     * Set the value of the field 'counter'
     */
    public void set_counter(int value) {
        setUIntBEElement(offsetBits_counter(), 16, value);
    }

    /**
     * Return the size, in bytes, of the field 'counter'
     */
    public static int size_counter() {
        return (16 / 8);
    }

    /**
     * Return the size, in bits, of the field 'counter'
     */
    public static int sizeBits_counter() {
        return 16;
    }

	//added by HyungJune Lee
	public int get_nodeid() {
		return (int)getUIntBEElement(0, 16);
	}

    public int get_seq() {
		int seq = ((dataGet()[2] & 0x00ff) << 24) | 
				  ((dataGet()[3] & 0x00ff) << 16) |
				  ((dataGet()[4] & 0x00ff) << 8) |
				  ((dataGet()[5] & 0x00ff) << 0); 
        return seq;
    }

    public int get_rssi() {
        return (dataGet()[6]-45);
    }

	public int get_timestamp() {
		return (int)getUIntBEElement(56, 32);
	}

	public int get_my_counter() {
		return (int)getUIntBEElement(88, 16);
	}

}
