/**
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT.  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */


/*
  http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
  Predefined: CRC16_ARC
  [x] Input reflected
  [x] Result reflected
  Polynomial: 0x8005
  Initial value: 0x0
  Final XOR value: 0x0

  NOTE: The '/' and '!' must be included in the crc calculation
*/

// CRC lookup table
unsigned short crctbl[256];

// Initialize the CRC lookup table
void initCrcTable()
{
  unsigned short crc;
  unsigned short c;
  unsigned int i;
  unsigned int j;
   
  for (i = 0; i < 256; i++)
  {
    crc = 0;
    c = i << 8;
      
    for (j = 0; j < 8; j++)
    {
      if ((crc ^ c) & 0x8000)
      {
         crc = (crc << 1) ^ 0x8005;
      }
      else
      {
         crc = (crc << 1);
      }
           
      c = (c << 1);
    }
    crctbl[i] = crc;
  }
}

// Reverse byte
unsigned char reverse(unsigned char b)
{
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

// Main CRC16-ARC calculation
unsigned short crc16(const char* data_p, unsigned short length)
{
  unsigned int i;
  unsigned short crc, crc_shifted;
  unsigned char c, tmp;

  // CRC initial value
  crc = 0x0;

  for (unsigned short px = 0; px < length; px++)
  {
    c = reverse(data_p[px]);
    tmp = ((crc >> 8) & 0xFF) ^ c;
    crc_shifted = (crc << 8) & 0xFF00;
    crc = crc_shifted ^ crctbl[tmp];
  }

  unsigned char lsb_rev, lsb;
  unsigned char lsb_byte;
  unsigned char msb_rev, msb;
  unsigned char msb_byte;
   
  lsb_rev = crc & 0x00FF;
  lsb = reverse(lsb_rev);
  lsb ^= 0x0; // Final XOR value lsb
   
  msb_rev = (crc & 0xFF00) >> 8;
  msb = reverse(msb_rev);
  msb ^= 0x0; // Final XOR value msb

  // Result reflected
  crc = (lsb << 8) + (msb);

  return crc;
}
