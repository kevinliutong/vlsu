package vlsu
import chisel3._
import chisel3.util._
/** Check requesting addr among lmshr and scalar dcache.
 * If miss, allocate a mshr.
 * If hit lmshr, do nothing.
 * If hit scalar dcache, padd data to wb-controller
 * */
class VectorLoadAddressChecker(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val brUpdate = Input(new BranchUpdateInfo(ap))
    /** Receive processing address from lmshrs */
    val lmshrStatus = Flipped(Vec(ap.nLmshrs, Valid(UInt(ap.coreMaxAddrBits.W))))
    /** requests from req-buffer winner. */
    val reqIncoming = Flipped(DecoupledIO(new VLdRequest(ap)))
    /** allocate an idle mshr if not hit any */
    val mshrAllocate = Vec(ap.nLmshrs, Valid(new VLdRequest(ap)))
    val vtlbResp = Flipped(Valid(new VTLBResp(ap)))
  })
  io.mshrAllocate := 0.U.asTypeOf(Vec(ap.nLmshrs, Valid(new VLdRequest(ap))))
  val mshrVlds = io.lmshrStatus.map(_.valid)

  val reg = RegInit(0.U.asTypeOf(Valid(new VLdRequest(ap))))
  val idleLmshrs = VecInit(io.lmshrStatus.map(!_.valid)).asUInt()
  val idleLmshr = idleLmshrs.orR()
  val allocateMshrIdx = OHToUInt(FindFirstOne(idleLmshrs, ap.nLmshrs))
}
