package vlsu

import chisel3._
import chisel3.util._
import boom.common.{BoomCoreParams, CSR_ADDR_SZ}
import chipsalliance.rocketchip.config.Parameters
import freechips.rocketchip.subsystem.CacheBlockBytes
import freechips.rocketchip.tile.XLen
import freechips.rocketchip.tilelink.{TLBundleParameters, TLEdgeOut}

case class VLSUGeneralParameters(nCacheLineSizeBytes: Int = 64,
                                 vLen: Int = 1024,
                                 nLmshrs: Int = 8,
                                 nSmshrs: Int = 8,
                                 nL2DataBeatBytes: Int = 32,
                                 nVLdQEntries: Int = 8,
                                 nVStQEntries: Int = 8,
                                 nVLdBuffEntries: Int = 128,
                                 nVStBuffEntries: Int = 128,
                                 coreWidth: Int = 2,
                                 nStoreWaitCycles: Int = 16,
                                 nVRegs: Int = 64
                                ){
  require(nL2DataBeatBytes < nCacheLineSizeBytes, "beat byte should small than cache line size.")
}

class VLSUArchitecturalParams(gp: VLSUGeneralParameters,
                              bp: BoomCoreParams,
                              edge: TLEdgeOut)(implicit p: Parameters){
  val coreWidth = gp.coreWidth
  val retireWidth = coreWidth

  val numRobRows = bp.numRobEntries / coreWidth
  val robAddrSz = log2Ceil(numRobRows) + log2Ceil(coreWidth)
  val coreMaxAddrBits = l2BundleParams.addressBits

  val nVLdQIdxBits = log2Ceil(gp.nVLdQEntries)
  val nVStQIdxBits = log2Ceil(gp.nVStQEntries)
  val nVLdQEntries = gp.nVLdQEntries
  val nVStQEntries = gp.nVStQEntries
  val nVLdBuffEntries = gp.nVLdBuffEntries
  val nVStBuffEntries = gp.nVStBuffEntries
  val nVLdBuffEntryIdxBits = log2Ceil(nVLdBuffEntries)
  val nVStBuffEntryIdxBits = log2Ceil(nVStBuffEntries)

  val maxBrCount = bp.maxBrCount

  val nCacheLineBytes: Int = gp.nCacheLineSizeBytes
  val nCacheLineBits: Int = nCacheLineBytes
  require(nCacheLineBytes == p(CacheBlockBytes))

  val l2BundleParams: TLBundleParameters = edge.bundle
  require(nCacheLineBits == l2BundleParams.dataBits, "one cache line per cycle. Use widthwidgt to adjust.")

  val tlEdge = edge

  val nVRegs = gp.nVRegs
  val nVPregSz = log2Ceil(nVRegs)
  val vLen = bp.vLen
  val xLen = p(XLen)
  val vLenb = vLen / 8
  /** Maximum elements number in one logical vector, which means eew = 8b and elmul = 8. */
  val vlMax = log2Ceil(vLen) + 1
  val vLenSz = log2Ceil(vLen)
  val elementMaxBytes = 8
  val maxReqsInUnitStride = (vLenb / nCacheLineBytes) + 1

  val addressBits = l2BundleParams.addressBits
  val offsetBits = log2Ceil(nCacheLineBytes)
  val offsetMSB = log2Ceil(nCacheLineBytes) - 1

}